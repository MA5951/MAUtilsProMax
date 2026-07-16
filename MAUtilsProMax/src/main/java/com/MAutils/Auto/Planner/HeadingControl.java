package com.MAutils.Auto.Planner;

import java.util.Currency;

import com.MAutils.Swerve.Utils.PIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class HeadingControl {

  public static final class Params {
    public double kP = 0.035 , kI = 0.0, kD = 0;
    public double maxVelRadPerSec = Math.toRadians(360.0);
    public double maxAccRadPerSec2 = Math.toRadians(720.0);
    public double toleranceRad = Math.toRadians(2.0);
    public double defaultR2 = 0.05;
  }

  // private final ProfiledPIDController pid;
  private final PIDController pidController;
  private final Params p;

  // --- Scheduling latch state ---
  private boolean scheduleActive = false;
  private double schedStartRad = 0.0;   // heading when we ENTER r1->r2 window
  private double schedDeltaRad = 0.0;   // shortest delta from start to final
  private double lastFinalGoalRad = 0.0;

  // For avoiding setGoal churn
  private double lastGoalRad = Double.NaN;

  public HeadingControl(Params p) {
    this.p = p;
    // this.pid = new ProfiledPIDController(
    //     p.kP, p.kI, p.kD,
    //     new TrapezoidProfile.Constraints(p.maxVelRadPerSec, p.maxAccRadPerSec2));
    // this.pid.enableContinuousInput(-Math.PI, Math.PI);
    // this.pid.setTolerance(p.toleranceRad);
    pidController = new PIDController(p.kP, p.kI, p.kD);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(p.toleranceRad);
  }

  public void configure(Params pNew) {
    // pid.setP(pNew.kP); pid.setI(pNew.kI); pid.setD(pNew.kD);
    // pid.setConstraints(new TrapezoidProfile.Constraints(pNew.maxVelRadPerSec, pNew.maxAccRadPerSec2));
    pidController.setP(pNew.kP);
    pidController.setI(pNew.kI);
    pidController.setD(pNew.kD);
  
  }

  public double omegaCommand(
      Rotation2d current,
      Rotation2d finalGoal,
      double distToXYGoal,
      double r1Start,
      double dtSeconds
  ) {
    

    //double omega = pid.calculate(curRad);
    
    double omega = pidController.calculate(current.getDegrees(),getRotationTarget(current, finalGoal, r1Start, distToXYGoal).getDegrees());

    return omega;
  }

  private double computeScheduledGoal(double curRad, double finalRad, double dist, double r1, double r2) {
    // Rotate ASAP
    if (r1 <= 0.0) {
      scheduleActive = false;
      return finalRad;
    }

    // Bad/degenerate window => rotate ASAP
    if (r1 <= r2) {
      scheduleActive = false;
      return finalRad;
    }

    // Outside window (too far): don't start yet
    if (dist >= r1) {
      scheduleActive = false;
      return curRad; // hold current (or you could return finalRad if you want pre-aiming)
    }

    // Enter window: latch start heading once
    if (!scheduleActive) {
      scheduleActive = true;
      schedStartRad = curRad;
      schedDeltaRad = wrapToPi(finalRad - schedStartRad);
    }

    // Progress 0..1 as dist goes r1 -> r2
    double progress;
    if (dist <= r2) progress = 1.0;
    else progress = (r1 - dist) / (r1 - r2);

    // Optional: smoother than linear (ease-in-out)
    progress = smoothstep(progress);

    return schedStartRad + schedDeltaRad * progress;
  }

  private static double smoothstep(double t) {
    t = Math.max(0.0, Math.min(1.0, t));
    return t * t * (3.0 - 2.0 * t);
  }

  private static double wrapToPi(double a) {
    while (a >  Math.PI) a -= 2.0 * Math.PI;
    while (a < -Math.PI) a += 2.0 * Math.PI;
    return a;
  }

  private Rotation2d getRotationTarget(Rotation2d current, Rotation2d target,double r1 ,double dist) {
    if (r1 == 0 || Double.isNaN(r1)) {
      return target;
    }
    if (dist < r1) {
      return target;
    } else {
      return current;
    }
  }
}
