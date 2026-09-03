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

        public Measurement interpolate(Measurement endValue, double t) {
            double interpolatedDx = MathUtil.interpolate(this.twist.dx, endValue.twist.dx, t);
            double interpolatedDy = MathUtil.interpolate(this.twist.dy, endValue.twist.dy, t);
            double interpolatedDtheta = MathUtil.interpolate(this.twist.dtheta, endValue.twist.dtheta, t);

            double interpolatedFomXY = MathUtil.interpolate(this.fomXY, endValue.fomXY, t);
            double interpolatedFomTheta = MathUtil.interpolate(this.fomTheta, endValue.fomTheta, t);
            double interpolatedTimestamp = MathUtil.interpolate(this.timestamp, endValue.timestamp, t);

            return new Measurement(
                new Twist2d(interpolatedDx, interpolatedDy, interpolatedDtheta),
                interpolatedFomXY,
                interpolatedFomTheta,
                interpolatedTimestamp
            );
        }
    }

    private static final double BUFFER_DURATION = 1.5; // seconds

    private final TimeInterpolatableBuffer<Measurement> buffer = 
            TimeInterpolatableBuffer.createBuffer((start, end, t) -> start.interpolate(end, t), 
            BUFFER_DURATION);

             // --- Suppliers ---
    private final Supplier<Twist2d> twistSupplier;
    private final Supplier<Double> fomXYSupplier;
    private final Supplier<Double> fomThetaSupplier;
    private final Supplier<Double> timestampSupplier;
    public final String name;

    // Primary ctor: separate XY and theta FOMs + explicit timestamp supplier
    public PoseEstimatorSource2(String name, Supplier<Twist2d> twistSupplier,
                               Supplier<Double> fomXYSupplier,
                               Supplier<Double> fomThetaSupplier,
                               Supplier<Double> timestampSupplier) {
        this.twistSupplier = twistSupplier;
        this.fomXYSupplier = fomXYSupplier;
        this.fomThetaSupplier = fomThetaSupplier;
        this.timestampSupplier = timestampSupplier;
        this.name = name;
    }

   public final void addMeasurement(Twist2d delta, double fomXY, double fomTheta, double timestamp) {
        if (fomXY <= 0) fomXY = Constants.MIN_FOM_VALUE;
        if (fomTheta <= 0) fomTheta = Constants.MIN_FOM_VALUE;
        Measurement p = new Measurement(delta, fomXY, fomTheta, timestamp);
        buffer.addSample(timestamp, p);
    }

   public void capture() {
        if (twistSupplier == null || fomXYSupplier == null || fomThetaSupplier == null) return;
        Twist2d delta = safeTwist(twistSupplier.get());
        double fxy = safePos(fomXYSupplier.get());
        double fth = safePos(fomThetaSupplier.get());
        double ts  = (timestampSupplier != null) ? timestampSupplier.get() : Timer.getFPGATimestamp();
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