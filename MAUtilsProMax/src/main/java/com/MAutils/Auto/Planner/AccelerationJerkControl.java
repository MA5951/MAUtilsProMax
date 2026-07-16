package com.MAutils.Auto.Planner;

import com.MAutils.Utils.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * AccelerationJerkControl — EXACT Autopilot-style velocity control.
 *
 * Matches Autopilot logic:
 *  - goalSpeed = cbrt(4.5 * dist^2 * jerk) + endVelocity
 *  - goalVec = dirUnit * goalSpeed (field frame)
 *  - correct(initialFieldVel, goalVec):
 *      rotate so goal points +X
 *      goalI = goalX (capped to maxSpeed)
 *      initialI = initialX
 *      adjustedI = min(goalI, push(initialI, goalI, accel))
 *      output = [adjustedI, 0] rotated back
 *
 * Notes:
 *  - Snap-down behavior is identical because of min(goalI, push(...)).
 *  - Uses fieldRelativeSpeeds = (vx,vy).rotateBy(poseRotation) like Autopilot.
 */
public final class AccelerationJerkControl {
  public static final class Params {
    public double maxSpeed = 3.8;      // m/s (Autopilot constraints.velocity)
    public double accel    = 7.0;      // m/s^2 (Autopilot constraints.acceleration)
    public double jerk     = 12.0;     // m/s^3 (Autopilot constraints.jerk)
    public double endVelocity = 0.0;   // m/s (Autopilot target.m_velocity)
  }

  private final Params p;

  // Debug / telemetry
  private double lastVTheory = 0.0;     // after maxSpeed cap
  private double lastVNowAlong = 0.0;   // projection of current field vel onto dir
  private double lastVCmd = 0.0;        // magnitude of commanded field vel

  public AccelerationJerkControl(Params p) {
    this.p = p;
  }

  /**
   * Compute the commanded FIELD-relative velocity vector using Autopilot math.
   *
   * @param pose Current pose (for robot->field velocity transform)
   * @param currentRobotSpeeds current ROBOT-relative speeds
   * @param dirFieldUnit desired direction of travel in FIELD frame (should be unit-ish)
   * @param remainingLength path length remaining (meters)
   * @param dtSeconds loop dt (Autopilot uses 0.020; if <=0, we use 0.020)
   */
  public Translation2d commandedFieldVelocity(
      Pose2d pose,
      ChassisSpeeds currentRobotSpeeds,
      Translation2d dirFieldUnit,
      double remainingLength,
      double dtSeconds
  ) {
    final double dt = (dtSeconds > 1e-6) ? dtSeconds : Constants.LOOP_TIME;

    // Autopilot: fieldRelativeSpeeds = (vx,vy).rotateBy(current.getRotation())
    Translation2d fieldRelativeSpeeds = new Translation2d(
        currentRobotSpeeds.vxMetersPerSecond,
        currentRobotSpeeds.vyMetersPerSecond
    ).rotateBy(pose.getRotation());

    // Normalize direction safely (Autopilot always uses a normalized "towardsTarget" or swirly unit)
    double dn = dirFieldUnit.getNorm();
    Translation2d dir = (dn > 1e-9)
        ? new Translation2d(dirFieldUnit.getX() / dn, dirFieldUnit.getY() / dn)
        : Translation2d.kZero;

    // Store "vNow along dir" (this matches Autopilot's adjustedInitial.getX() after rotation)
    lastVNowAlong = Math.max(0.0, fieldRelativeSpeeds.getX() * dir.getX() + fieldRelativeSpeeds.getY() * dir.getY());

    // Autopilot calculateMaxVelocity(dist, endVelo), then cap by constraints.velocity
    double vTheory = calculateMaxVelocity(remainingLength, p.endVelocity);
    if (vTheory > p.maxSpeed) vTheory = p.maxSpeed;
    lastVTheory = vTheory;

    // goal vector in field frame
    Translation2d goal = (dir.equals(Translation2d.kZero))
        ? Translation2d.kZero
        : dir.times(vTheory);

    // Autopilot correct(initial, goal)
    Translation2d out = correct(fieldRelativeSpeeds, goal, p.accel, dt, p.maxSpeed);

    lastVCmd = out.getNorm();
    return out;
  }

  /** Autopilot exact: v = cbrt(4.5 * dist^2 * jerk) + endVelo */
  private double calculateMaxVelocity(double dist, double endVelo) {
    dist = Math.max(0.0, dist);
    return Math.cbrt((4.5 * dist * dist) * p.jerk) + endVelo;
  }

  /** Autopilot exact correct() */
  private static Translation2d correct(Translation2d initial, Translation2d goal, double accel, double dt, double vCap) {
    Rotation2d angleOffset = Rotation2d.kZero;
    if (!goal.equals(Translation2d.kZero)) {
      angleOffset = new Rotation2d(goal.getX(), goal.getY());
    }

    Translation2d adjustedGoal = goal.rotateBy(angleOffset.unaryMinus());
    Translation2d adjustedInitial = initial.rotateBy(angleOffset.unaryMinus());

    double initialI = adjustedInitial.getX();
    double goalI = adjustedGoal.getX();

    // Autopilot caps goalI to constraints.velocity
    if (goalI > vCap) goalI = vCap;

    // Autopilot: adjustedI = min(goalI, push(initialI, goalI, accel))
    double adjustedI = Math.min(goalI, push(initialI, goalI, accel, dt));

    return new Translation2d(adjustedI, 0.0).rotateBy(angleOffset);
  }

  /** Autopilot exact push() */
  private static double push(double start, double end, double accel, double dt) {
    double maxChange = accel * dt;
    if (Math.abs(start - end) < maxChange) return end;
    if (start > end) return start - maxChange;
    return start + maxChange;
  }

  public double lastVTheory() { return lastVTheory; }
  public double lastVNowAlong() { return lastVNowAlong; }
  public double lastVCmd() { return lastVCmd; }
}
