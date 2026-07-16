package com.MAutils.Auto.Planner;

import java.util.List;

import com.MAutils.Logger.MALog;
import com.MAutils.Utils.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * MAPlanner — moving-target holonomic controller with single-circle avoidance.
 *
 * Adds optional "entry angle" behavior (Autopilot-style) when LOS is clear and
 * within a configurable radius near the target.
 *
 * Output: FIELD-relative ChassisSpeeds (vx, vy, ω). Convert to robot frame at
 * drivetrain if needed.
 */
public final class MAPlanner {
  // ----------------------- Public Types & Params -----------------------

  /** Circular obstacle type (kept here so external code can reuse it). */
  public static final class CircleObstacle {
    public final double x, y, r;

    public CircleObstacle(double x, double y, double r) {
      this.x = x;
      this.y = y;
      this.r = r;
    }
  }

  /**
   * Optional entry-angle shaping (translation direction shaping), only near
   * target.
   */
  public static final class EntryParams {
    /**
     * Field-relative direction you want to ARRIVE FROM (the +X axis of the target
     * frame).
     */
    public final Rotation2d entryDirection;
    /** Only apply entry shaping when distToGoal <= radius (+blend) (m). */
    public final double radius;

    /** Blend width (m). 0 = hard switch at radius. Suggest 0.2-0.6m */

    public EntryParams(Rotation2d entryDirection) {
      this.entryDirection = entryDirection;
      this.radius = 0.3;
    }
  }

  public static final class Params {
    /** Max linear speed (m/s). */
    public double maxSpeed = 3.3;
    /** Considered "at goal" distance (m). */
    public double stopRadius = 0.05;
    /** Extra safety margin added to circle radius for LOS checks (m). */
    public double margin = 0.0; // keep 0.0 if circles are already buffered

    // Motion params (delegated to AccelerationJerkControl)
    public double accel = 19; // m/s^2
    public double jerk = 12; // m/s^3
    public double endVelocity = 0.0; // m/s
  }

  public static final class Debug {
    public Pose2d guidePose = new Pose2d();
    public double usedStepMeters = 0.0;
    public String mode = "FREE"; // FREE / GLIDE_CW / GLIDE_CCW
    public Translation2d entryPoint = null;
    public Translation2d exitPoint = null;
    public CircleObstacle hit = null;

    // Velocity debug
    public double vTheoretical = 0.0;
    public double vCommand = 0.0;

    // Entry-angle debug
    public boolean entryActive = false;
    public double entryBlendW = 0.0;

    public void logDebug() {
      MALog.log("MAPlanner/Mode", mode);
      MALog.log("MAPlanner/GuidePose", guidePose);

      if (entryPoint != null)
        MALog.log("MAPlanner/EntryPoint/", new Pose2d(entryPoint, new Rotation2d()));
      if (exitPoint != null)
        MALog.log("MAPlanner/ExitPoint/", new Pose2d(exitPoint, new Rotation2d()));

      if (hit != null) {
        MALog.log("MAPlanner/HitCircle/X", hit.x);
        MALog.log("MAPlanner/HitCircle/Y", hit.y);
        MALog.log("MAPlanner/HitCircle/R", hit.r);
      }

      MALog.log("MAPlanner/UsedStepMeters", usedStepMeters);
      MALog.log("MAPlanner/vTheoretical", vTheoretical);
      MALog.log("MAPlanner/vCommand", vCommand);

      MALog.log("MAPlanner/Entry/Active", entryActive);
      MALog.log("MAPlanner/Entry/BlendW", entryBlendW);
    }
  }

  // ----------------------- State -----------------------

  private final Params p;
  private final HeadingControl headingCtl;
  public final Debug dbg = new Debug();
  private final AccelerationJerkControl speedCtl;

  // Latch to avoid side flipping mid-glide
  private CircleObstacle latchedCircle = null;
  private boolean latchedCCW = true; // true = CCW arc

  // If caller doesn't give measured speed, reuse last commanded as estimate
  private double lastCmdSpeed = 0.0;
  private double distToGoal = Double.POSITIVE_INFINITY;

  public MAPlanner(Params p) {
    this.p = p;

    AccelerationJerkControl.Params mp = new AccelerationJerkControl.Params();
    mp.maxSpeed = p.maxSpeed;
    mp.accel = p.accel;
    mp.jerk = p.jerk;
    mp.endVelocity = p.endVelocity;
    this.speedCtl = new AccelerationJerkControl(mp);

    HeadingControl.Params hp = new HeadingControl.Params();
    hp.defaultR2 = p.stopRadius;
    this.headingCtl = new HeadingControl(hp);
  }

  public double getDistanceToGoal() {
    return distToGoal;
  }

  public double getStopRadius() {
    return p.stopRadius;
  }

  // ----------------------- Legacy Pose Guide API -----------------------

  /** Compute next guide pose with obstacle circles (original behavior). */
  public Pose2d update(Pose2d currentPose, Translation2d goalXY, List<CircleObstacle> circles, double dtSeconds) {
    Translation2d S = currentPose.getTranslation();
    double maxStep = Math.max(0.0, p.maxSpeed * Math.max(0.0, dtSeconds));

    // If already at goal
    double distToGoal = S.getDistance(goalXY);
    if (distToGoal <= p.stopRadius) {
      Pose2d out = new Pose2d(goalXY, currentPose.getRotation());
      dbg.mode = "FREE";
      dbg.entryPoint = dbg.exitPoint = null;
      dbg.hit = null;
      dbg.usedStepMeters = 0.0;
      dbg.guidePose = out;
      dbg.entryActive = false;
      dbg.entryBlendW = 0.0;
      dbg.logDebug();
      return out;
    }

    // Choose closest circle that blocks the full LOS (S->goal)
    BlockHit hit = pickClosestBlocking(S, goalXY, circles, p.margin);

    Translation2d G;
    if (hit == null) {
      // LOS is clear → advance straight toward goal
      G = Geometry.advanceAlongLine(S, goalXY, maxStep);
      dbg.mode = "FREE";
      dbg.entryPoint = dbg.exitPoint = null;
      dbg.hit = null;
      latchedCircle = null; // release latch
    } else {
      // Build glide path using tangents. If switching circles, re-evaluate side.
      GlidePath gp;
      if (latchedCircle != null && latchedCircle == hit.circle) {
        gp = glidePathWithFixedSide(S, goalXY, hit.circle, latchedCCW);
      } else {
        gp = bestGlidePath(S, goalXY, hit.circle);
        latchedCircle = hit.circle;
        latchedCCW = gp.ccw;
      }

      // Take a step of length maxStep along the composite path
      G = firstAlongComposite(S, gp, goalXY, maxStep);
      dbg.mode = gp.ccw ? "GLIDE_CCW" : "GLIDE_CW";
      dbg.entryPoint = gp.entry;
      dbg.exitPoint = gp.exit;
      dbg.hit = hit.circle;
    }

    Pose2d out = new Pose2d(G, currentPose.getRotation());
    dbg.guidePose = out;
    dbg.usedStepMeters = Math.min(maxStep, distToGoal);
    dbg.entryActive = false;
    dbg.entryBlendW = 0.0;
    dbg.logDebug();
    return out;
  }

  /** Overload with dt=0.02s. */
  public Pose2d update(Pose2d currentPose, Translation2d goalXY, List<CircleObstacle> circles) {
    return update(currentPose, goalXY, circles, Constants.LOOP_TIME);
  }

  // ----------------------- Speeds API (field-relative) -----------------------

  /**
   * Field-relative speeds; pass current robot-relative speeds for exact accel
   * limiting.
   */
  public ChassisSpeeds updateSpeeds(
      Pose2d pose,
      Translation2d goalXY,
      HeadingParams heading,
      EntryParams entry,
      List<CircleObstacle> circles,
      double dtSeconds,
      ChassisSpeeds currentRobotSpeeds) {
    Translation2d S = pose.getTranslation();
    double distToGoal = S.getDistance(goalXY);

    // If already at goal: stop translation, still allow heading controller to
    // finish
    if (distToGoal <= p.stopRadius) {
      dbg.vTheoretical = 0.0;
      dbg.vCommand = 0.0;
      dbg.usedStepMeters = 0.0;
      dbg.guidePose = new Pose2d(goalXY, pose.getRotation());
      dbg.mode = "AT GOAL";
      dbg.entryPoint = dbg.exitPoint = null;
      dbg.hit = null;
      dbg.entryActive = false;
      dbg.entryBlendW = 0.0;

      double omega = 0.0;
      if (heading != null) {
        omega = headingCtl.omegaCommand(
            pose.getRotation(),
            heading.targetRotation,
            distToGoal,
            heading.r1Start,
            dtSeconds);
      }

      dbg.logDebug();
      lastCmdSpeed = 0.0;
      return new ChassisSpeeds(0.0, 0.0, omega);
    }

    // Pick closest blocking circle (same logic)
    boolean wantSwirly = (entry != null && entry.entryDirection != null && distToGoal >= entry.radius);

    BlockHit hit = wantSwirly
        ? pickClosestBlockingSwirly(S, goalXY, entry.entryDirection, circles, p.margin)
        : pickClosestBlocking(S, goalXY, circles, p.margin);

    // Compute remaining path length L and unit direction
    double L;
    Translation2d dirField;

    dbg.entryActive = false;
    dbg.entryBlendW = 0.0;

    if (hit == null) {
      // Straight-to-goal baseline
      Translation2d toGoal = goalXY.minus(S);
      double disp = toGoal.getNorm();
      Translation2d beelineDir = (disp > 1e-9)
          ? new Translation2d(toGoal.getX() / disp, toGoal.getY() / disp)
          : new Translation2d(0.0, 0.0);

      // Default: beeline
      dirField = beelineDir;
      L = disp;

      dbg.mode = "NO HIT - BEELINE";
      dbg.entryPoint = dbg.exitPoint = null;
      dbg.hit = null;
      latchedCircle = null;

      // -------- Autopilot-style entry logic (EXACT behavior) --------
      // Autopilot: if no entryAngle OR disp < beelineRadius -> beeline
      // else -> swirly
      if (entry != null && entry.entryDirection != null) {

        // entry.radius == Autopilot's beelineRadius
        boolean useSwirly = disp >= entry.radius;

        if (useSwirly) {
          dbg.mode = "NO HIT - ENTRY";
          dirField = autopilotSwirlyDirField(S, goalXY, entry.entryDirection);
          L = autopilotSwirlyRemainingLengthMeters(S, goalXY, entry.entryDirection);

          dbg.entryActive = true;
          dbg.entryBlendW = 1.0;
        } else {
          dbg.entryActive = false;
          dbg.entryBlendW = 0.0;
        }
      } else {
        dbg.entryActive = false;
        dbg.entryBlendW = 0.0;
      }

    } else {
      // Composite: straight→arc→straight
      GlidePath gp;
      if (latchedCircle != null && latchedCircle == hit.circle) {
        gp = glidePathWithFixedSide(S, goalXY, hit.circle, latchedCCW);
      } else {
        gp = bestGlidePath(S, goalXY, hit.circle);
        latchedCircle = hit.circle;
        latchedCCW = gp.ccw;
      }
      L = remainingCompositeLength(S, gp, goalXY);

      // Direction = tangent of the path at the current position toward the next piece
      double len1 = S.getDistance(gp.entry);
      if (len1 > 1e-6) {
        Translation2d toEntry = gp.entry.minus(S);
        double d = toEntry.getNorm();
        dirField = (d > 1e-9)
            ? new Translation2d(toEntry.getX() / d, toEntry.getY() / d)
            : new Translation2d(0.0, 0.0);
      } else {
        double sin = Math.sin(gp.angEntry), cos = Math.cos(gp.angEntry);
        Translation2d tanAtEntry = gp.ccw ? new Translation2d(-sin, cos) : new Translation2d(sin, -cos);
        boolean losBlocked = blocksLOS(S, goalXY, hit.circle, p.margin);
        if (losBlocked) {
          dirField = tanAtEntry;
        } else {
          Translation2d toGoal = goalXY.minus(gp.exit);
          double d = toGoal.getNorm();
          dirField = (d > 1e-9)
              ? new Translation2d(toGoal.getX() / d, toGoal.getY() / d)
              : new Translation2d(0.0, 0.0);
        }
      }

      dbg.mode = gp.ccw ? "GLIDE_CCW" : "GLIDE_CW";
      dbg.entryPoint = gp.entry;
      dbg.exitPoint = gp.exit;
      dbg.hit = hit.circle;
    }

    // Current velocity along path tangent (field frame)
    Translation2d vField = speedCtl.commandedFieldVelocity(pose, currentRobotSpeeds, dirField, L, dtSeconds);

    MALog.log("MAPlanner/vNow", speedCtl.lastVNowAlong());
    double vCmd = speedCtl.lastVCmd();

    // Build field-relative velocity vector

    double omega = 0.0;
    if (heading != null) {
      omega = headingCtl.omegaCommand(
          pose.getRotation(),
          heading.targetRotation,
          distToGoal,
          heading.r1Start,
          dtSeconds);
    }

    // Debug & guide
    Translation2d guideXY = new Translation2d(
        S.getX() + vField.getX() * Math.max(0.0, dtSeconds),
        S.getY() + vField.getY() * Math.max(0.0, dtSeconds));

    dbg.guidePose = new Pose2d(guideXY, pose.getRotation());
    dbg.vTheoretical = speedCtl.lastVTheory();
    dbg.vCommand = vCmd;
    dbg.usedStepMeters = vCmd * Math.max(0.0, dtSeconds);
    dbg.logDebug();

    lastCmdSpeed = vCmd;
    return new ChassisSpeeds(vField.getX(), vField.getY(), omega); // FIELD-relative
  }

  // ----------------------- Entry-angle helpers (Autopilot-style spiral b=1)
  // -----------------------

  /** Direction of inward spiral in FIELD frame (unit vector). */
  private static Translation2d autopilotSwirlyDirField(
      Translation2d robotXY,
      Translation2d goalXY,
      Rotation2d entryDirectionField) {
    // offset from robot -> goal (FIELD frame) (this matches Autopilot)
    Translation2d offsetField = goalXY.minus(robotXY);

    // rotate into target frame (+X is entry direction)
    Translation2d offsetTarget = offsetField.rotateBy(entryDirectionField.unaryMinus());

    if (offsetTarget.equals(Translation2d.kZero)) {
      return new Translation2d(0.0, 0.0);
    }

    Rotation2d theta = new Rotation2d(offsetTarget.getX(), offsetTarget.getY());
    double rads = theta.getRadians();

    // Autopilot formulas
    double vx = theta.getCos() - rads * theta.getSin();
    double vy = rads * theta.getCos() + theta.getSin();

    double n = Math.hypot(vx, vy);
    if (n < 1e-9)
      return new Translation2d(0.0, 0.0);

    Translation2d dirTarget = new Translation2d(vx / n, vy / n);

    // rotate back to FIELD frame
    return dirTarget.rotateBy(entryDirectionField);
  }

  /** Autopilot exact swirly arc-length (matches calculateSwirlyLength). */
  private static double autopilotSwirlyLength(double thetaRad, double radiusMeters) {
    if (thetaRad == 0.0)
      return radiusMeters;
    thetaRad = Math.abs(thetaRad);
    double hypot = Math.hypot(thetaRad, 1.0);
    double u1 = radiusMeters * hypot;
    double u2 = radiusMeters * Math.log(thetaRad + hypot) / thetaRad;
    return 0.5 * (u1 + u2);
  }

  /** Convenience: swirly remaining length from current robot->goal offset. */
  private static double autopilotSwirlyRemainingLengthMeters(
      Translation2d robotXY,
      Translation2d goalXY,
      Rotation2d entryDirectionField) {
    Translation2d offsetField = goalXY.minus(robotXY);
    double disp = offsetField.getNorm();
    if (disp < 1e-9)
      return 0.0;

    Translation2d offsetTarget = offsetField.rotateBy(entryDirectionField.unaryMinus());
    Rotation2d theta = new Rotation2d(offsetTarget.getX(), offsetTarget.getY());
    return autopilotSwirlyLength(theta.getRadians(), disp);
  }

  // ----------------------- Geometry / Path Core (unchanged behavior)
  // -----------------------

  private static final class BlockHit {
    final CircleObstacle circle;
    final double t;

    BlockHit(CircleObstacle c, double t) {
      this.circle = c;
      this.t = t;
    }
  }

  private BlockHit pickClosestBlocking(Translation2d S, Translation2d T, List<CircleObstacle> circles, double margin) {
    BlockHit best = null;
    double bestT = Double.POSITIVE_INFINITY;
    for (CircleObstacle c : circles) {
      Geometry.IntersectResult ir = Geometry.segmentCircleIntersect(S, T, new Translation2d(c.x, c.y), c.r - 0.55);
      if (ir.hit && ir.tHit < bestT) {
        bestT = ir.tHit;
        best = new BlockHit(c, ir.tHit);
      }
    }
    return best;
  }

  /**
   * Picks earliest circle hit along the AUTOPILOT swirly path (polyline
   * approximation).
   */
  private BlockHit pickClosestBlockingSwirly(
      Translation2d start,
      Translation2d goal,
      Rotation2d entryDirField,
      List<CircleObstacle> circles,
      double margin) {
    // Tune these:
    final double stepMeters = 0.05; // 5cm sampling
    final int maxSteps = 300; // max polyline segments
    final double maxTravel = start.getDistance(goal) * 2.0 + 1.0; // safety cap

    Translation2d p0 = start;
    double traveled = 0.0;

    BlockHit best = null;
    double bestS = Double.POSITIVE_INFINITY;

    for (int i = 0; i < maxSteps; i++) {
      double distLeft = p0.getDistance(goal);
      if (distLeft <= 1e-6)
        break;

      double ds = Math.min(stepMeters, distLeft);
      Translation2d dir = autopilotSwirlyDirField(p0, goal, entryDirField);
      double dn = dir.getNorm();
      if (dn < 1e-9)
        break;

      // Advance one step along the swirly direction
      Translation2d p1 = new Translation2d(
          p0.getX() + dir.getX() / dn * ds,
          p0.getY() + dir.getY() / dn * ds);

      // Check all circles for intersection with this segment
      for (CircleObstacle c : circles) {
        Geometry.IntersectResult ir = Geometry.segmentCircleIntersect(
            p0, p1, new Translation2d(c.x, c.y), c.r - 0.55);

        if (ir.hit) {
          double sHit = traveled + ds * ir.tHit; // hit distance along the polyline
          if (sHit < bestS) {
            bestS = sHit;
            best = new BlockHit(c, sHit); // store sHit in the existing "t" field
          }
        }
      }

      traveled += ds;
      if (traveled > maxTravel)
        break;
      p0 = p1;
    }

    return best;
  }

  private boolean blocksLOS(Translation2d S, Translation2d T, CircleObstacle c, double margin) {
    return Geometry.segmentCircleIntersect(S, T, new Translation2d(c.x, c.y), c.r + margin).hit;
  }

  private static final class GlidePath {
    final boolean ccw;
    final Translation2d entry, exit;
    final double angEntry, angExit, R;
    final Translation2d C;

    GlidePath(boolean ccw, Translation2d entry, Translation2d exit, double angEntry, double angExit, double R,
        Translation2d C) {
      this.ccw = ccw;
      this.entry = entry;
      this.exit = exit;
      this.angEntry = angEntry;
      this.angExit = angExit;
      this.R = R;
      this.C = C;
    }
  }

  private static final class Candidate {
    final boolean ccw;
    final double totalLen;
    final Translation2d entry, exit;
    final double angEntry, angExit, R;

    Candidate(boolean ccw, double totalLen, Translation2d entry, Translation2d exit, double angEntry, double angExit,
        double R) {
      this.ccw = ccw;
      this.totalLen = totalLen;
      this.entry = entry;
      this.exit = exit;
      this.angEntry = angEntry;
      this.angExit = angExit;
      this.R = R;
    }

    GlidePath toGlidePath(Translation2d C) {
      return new GlidePath(ccw, entry, exit, angEntry, angExit, R, C);
    }
  }

  private GlidePath bestGlidePath(Translation2d S, Translation2d T, CircleObstacle co) {
    double R = co.r + p.margin;
    Translation2d C = new Translation2d(co.x, co.y);
    Geometry.TanPair sTan = Geometry.tangentAngles(S, C, R);
    Geometry.TanPair tTan = Geometry.tangentAngles(T, C, R);
    Candidate ccw = buildCandidate(S, T, C, R, sTan.plus, tTan.plus, true);
    Candidate cw = buildCandidate(S, T, C, R, sTan.minus, tTan.minus, false);
    return (ccw.totalLen <= cw.totalLen) ? ccw.toGlidePath(C) : cw.toGlidePath(C);
  }

  private GlidePath glidePathWithFixedSide(Translation2d S, Translation2d T, CircleObstacle co, boolean ccw) {
    double R = co.r + p.margin;
    Translation2d C = new Translation2d(co.x, co.y);
    Geometry.TanPair sTan = Geometry.tangentAngles(S, C, R);
    Geometry.TanPair tTan = Geometry.tangentAngles(T, C, R);
    double angEntry = ccw ? sTan.plus : sTan.minus;
    double angExit = ccw ? tTan.plus : tTan.minus;
    Translation2d entry = Geometry.fromAngle(C, R, angEntry);
    Translation2d exit = Geometry.fromAngle(C, R, angExit);
    return new GlidePath(ccw, entry, exit, angEntry, angExit, R, C);
  }

  private Candidate buildCandidate(Translation2d S, Translation2d T, Translation2d C, double R,
      double angEntry, double angExit, boolean ccw) {
    Translation2d entry = Geometry.fromAngle(C, R, angEntry);
    Translation2d exit = Geometry.fromAngle(C, R, angExit);
    double len1 = S.getDistance(entry);
    double arc = Geometry.arcLen(R, angEntry, angExit, ccw);
    double len3 = exit.getDistance(T);
    return new Candidate(ccw, len1 + arc + len3, entry, exit, angEntry, angExit, R);
  }

  private static Translation2d firstAlongComposite(Translation2d S, GlidePath gp, Translation2d T, double s) {
    double len1 = S.getDistance(gp.entry);
    double len2 = Geometry.arcLen(gp.R, gp.angEntry, gp.angExit, gp.ccw);
    double sClamped = Math.max(0.0, s);

    if (sClamped <= len1)
      return Geometry.advanceAlongLine(S, gp.entry, sClamped);
    sClamped -= len1;

    if (sClamped <= len2) {
      double ang = gp.angEntry + (gp.ccw ? 1 : -1) * (sClamped / Math.max(1e-9, gp.R));
      return Geometry.fromAngle(gp.C, gp.R, ang);
    }
    sClamped -= len2;

    return Geometry.advanceAlongLine(gp.exit, T, sClamped);
  }

  private static double remainingCompositeLength(Translation2d S, GlidePath gp, Translation2d T) {
    double len1 = S.getDistance(gp.entry);
    double len2 = Geometry.arcLen(gp.R, gp.angEntry, gp.angExit, gp.ccw);
    double len3 = gp.exit.getDistance(T);
    return len1 + len2 + len3;
  }
}
