// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.MAutils.PoseEstimation.Helpers;

import edu.wpi.first.math.geometry.Twist2d;

/** Add your docs here. */
public class HistoryEntry {
    public final double time;
    public final Twist2d twist;

        public HistoryEntry(double t, Twist2d tw) {
            time = t;
            twist = tw;
        }
}
