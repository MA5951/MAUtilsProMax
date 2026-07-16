package com.MAutils.Auto.Planner;

import edu.wpi.first.math.geometry.Rotation2d;

/** HeadingParams: user-facing input for heading scheduling. */
public final class HeadingParams {
  public final Rotation2d targetRotation;
  /** r1: start rotation when dist <= r1; if null or <=0: start ASAP */
  public final Double r1Start;
  /** r2: must be complete when dist <= r2; if null: defaults to planner stop radius (via HeadingControl.Params.defaultR2) */

  public HeadingParams(Rotation2d targetRotation, double r1Start) {
    this.targetRotation = targetRotation;
    this.r1Start = r1Start;
  }


    
}
