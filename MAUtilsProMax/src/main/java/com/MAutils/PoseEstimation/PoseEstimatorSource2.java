package com.MAutils.PoseEstimation;

import java.util.Comparator;
import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.Utils.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;

/*
 * Represents a source of pose change measurements for a pose estimator.
 */
public class PoseEstimatorSource2 {

    public static class Measurement {
        final Twist2d twist;
        final double fomXY, fomTheta, timestamp;

        Measurement(Twist2d t, double fXY, double fTh, double ts) {
            twist = t; 
            fomXY = fXY; 
            fomTheta = fTh; 
            timestamp = ts;
        }

    }

    private static final double BUFFER_DURATION = 1.5; // seconds
    private Twist2d interpolatedTwist = new Twist2d();

    private final TimeInterpolatableBuffer<Measurement> buffer = 
    TimeInterpolatableBuffer.createBuffer(
        (startValue, endValue, t) -> {
            interpolatedTwist = new Twist2d(
                MathUtil.interpolate(startValue.twist.dx, endValue.twist.dx, t),
                MathUtil.interpolate(startValue.twist.dy, endValue.twist.dy, t),
                MathUtil.interpolate(startValue.twist.dtheta, endValue.twist.dtheta, t)
            );

            double interpolatedFomXY = MathUtil.interpolate(startValue.fomXY, endValue.fomXY, t);
            double interpolatedFomTheta = MathUtil.interpolate(startValue.fomTheta, endValue.fomTheta, t);
            double interpolatedTimestamp = MathUtil.interpolate(startValue.timestamp, endValue.timestamp, t);

            return new Measurement(
                interpolatedTwist, 
                interpolatedFomXY, 
                interpolatedFomTheta, 
                interpolatedTimestamp
            );
        }, 
        BUFFER_DURATION
    );

    
    private final Twist2d twist2d;
    private final Double fomXY;
    private final Double fomTheta;
    private final Double timestamp;
    public final String name;

    // Primary ctor: separate XY and theta FOMs + explicit timestamp supplier
    public PoseEstimatorSource2(String name, Twist2d twist2d,
                               Double fomXY,
                               Double fomTheta,
                               Double timestamp) {
        this.twist2d = twist2d;
        this.fomXY = fomXY;
        this.fomTheta = fomTheta;
        this.timestamp = timestamp;
        this.name = name;
    }

   public final void addMeasurement(Twist2d delta, double fomXY, double fomTheta, double timestamp) {
        if (fomXY <= Constants.MIN_FOM_VALUE) fomXY = Constants.MIN_FOM_VALUE;
        if (fomTheta <= Constants.MIN_FOM_VALUE) fomTheta = Constants.MIN_FOM_VALUE;
        Measurement p = new Measurement(delta, fomXY, fomTheta, timestamp);
        buffer.addSample(timestamp, p);
    }

   public void capture() {
        if (twist2d == null || fomXY == null || fomTheta == null) return;
        Twist2d delta = safeTwist(twist2d);
        double fxy = safePos(fomXY);
        double fth = safePos(fomTheta);
        double ts  = (timestamp != null) ? timestamp : Timer.getFPGATimestamp();
        MALog.log("Pose Estimator/Sources/"+ name +"/Twist/X", delta.dx);
        MALog.log("Pose Estimator/Sources/"+ name +"/Twist/Y", delta.dy);
        MALog.log("Pose Estimator/Sources/"+ name +"/Twist/Theta", delta.dtheta);
        MALog.log("Pose Estimator/Sources/"+ name +"/FOM XY", fxy);
        MALog.log("Pose Estimator/Sources/"+ name +"/FOM Theta", fth);
        MALog.log("Pose Estimator/Sources/"+ name +"/Timestemp", ts);
        addMeasurement(delta, fxy, fth, ts);
    }

    private Twist2d safeTwist(Twist2d t) { 
        if (t == null) return new Twist2d(0.0, 0.0, 0.0); 
        double dx = Double.isFinite(t.dx) ? t.dx : 0.0;
        double dy = Double.isFinite(t.dy) ? t.dy : 0.0;
        double dth = Double.isFinite(t.dtheta) ? t.dtheta : 0.0;
        return new Twist2d(dx, dy, dth);
    }

    private double safePos(Double v) {
        if (v == null || !Double.isFinite(v) || v <= 0.0) return Constants.MIN_FOM_VALUE;
        return v;
    }

    public Measurement getMeasurement(double timestamp) {
        return buffer.getSample(timestamp).orElse(null);
    }
}