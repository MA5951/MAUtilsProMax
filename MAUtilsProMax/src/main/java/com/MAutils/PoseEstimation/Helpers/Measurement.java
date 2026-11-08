
package com.MAutils.PoseEstimation.Helpers;

import edu.wpi.first.math.geometry.Twist2d;

public class Measurement {

    public final Twist2d twist;
    public final double fomXY, fomTheta, timestamp;

        public Measurement(Twist2d t, double fXY, double fTh, double ts) {
            twist = t; 
            fomXY = fXY; 
            fomTheta = fTh; 
            timestamp = ts;
        }
}
