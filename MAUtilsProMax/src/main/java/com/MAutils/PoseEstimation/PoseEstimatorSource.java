package com.MAutils.PoseEstimation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.Utils.Constants;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;

/*
 * Represents a source of pose change measurements for a pose estimator.
 */

public class PoseEstimatorSource {
    private static class Measurement {
        final Twist2d twist;
        final double fomXY, fomTheta, timestamp;
        Measurement(Twist2d t, double fXY, double fTh, double ts) {
            twist = t; fomXY = fXY; fomTheta = fTh; timestamp = ts;
        }
    }
 //TODO it will be better to limit the num of varubal in the array list and not save all of them
 // also then you wont need to creat a new Measurement every time
    private final List<Measurement> buffer = new ArrayList<>();

    // --- Suppliers ---
    private final Supplier<Twist2d> twistSupplier;
    private final Supplier<Double> fomXYSupplier;
    private final Supplier<Double> fomThetaSupplier;
    private final Supplier<Double> timestampSupplier;
    public final String name;

    // Primary ctor: separate XY and theta FOMs + explicit timestamp supplier
    public PoseEstimatorSource(String name, Supplier<Twist2d> twistSupplier,
                               Supplier<Double> fomXYSupplier,
                               Supplier<Double> fomThetaSupplier,
                               Supplier<Double> timestampSupplier) {
        this.twistSupplier = twistSupplier;
        this.fomXYSupplier = fomXYSupplier;
        this.fomThetaSupplier = fomThetaSupplier;
        this.timestampSupplier = timestampSupplier;
        this.name = name;
    }

    /** Legacy no-arg ctor kept for backward compatibility (manual pushes). */
    public PoseEstimatorSource(String name) {
        this.twistSupplier = null;
        this.fomXYSupplier = null;
        this.fomThetaSupplier = null;
        this.timestampSupplier = null;
        this.name = name;
    }

    /** Backward-compatible manual push: same FOM to XY and θ. */
    public void addMeasurement(Twist2d delta, double fom, double timestamp) {
        addMeasurement(delta, fom, fom, timestamp);
    }

    /** Backward-compatible manual push: separate XY/θ FOMs. */
    public  void addMeasurement(Twist2d delta, double fomXY, double fomTheta, double timestamp) {
        if (fomXY <= 0) fomXY = Constants.EPSILON;
        if (fomTheta <= 0) fomTheta = Constants.EPSILON;
        Measurement p = new Measurement(delta, fomXY, fomTheta, timestamp);
        int idx = java.util.Collections.binarySearch(buffer, p, Comparator.comparingDouble(x -> x.timestamp));
        if (idx < 0) idx = -idx - 1;
        buffer.add(idx, p);
    }

    /** New: read from suppliers and append a measurement. */
    public  void capture() {
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

    /** Alias for capture(), if you prefer the naming. */
    public void addFromSuppliers() { capture(); }

    private static Twist2d safeTwist(Twist2d t) { 
        if (t == null) return new Twist2d(); //TODO return epsilon instnde the robot move you just dont whant to consider it
        double dx = Double.isFinite(t.dx) ? t.dx : 0.0;
        double dy = Double.isFinite(t.dy) ? t.dy : 0.0;
        double dth = Double.isFinite(t.dtheta) ? t.dtheta : 0.0;
        return new Twist2d(dx, dy, dth);
    }

    //TODO why you need to check if V in null and why you use the double class?  
    private static double safePos(Double v) {
        if (v == null || !Double.isFinite(v) || v <= 0.0) return Constants.EPSILON;
        return v;
    }

    public boolean hasBefore(double t) {
        return !buffer.isEmpty() && buffer.get(0).timestamp < t;
    }

    public  Twist2d getTwistAt(double t) {
        //TODO if you got a big latnce the <= can be a problem its beeter to add a threshold to the serch
        for (int i = buffer.size() - 1; i >= 0; i--) {
            if (buffer.get(i).timestamp <= t) {
                return buffer.get(i).twist;
            }
        }
        return new Twist2d();
    }

    /** Overall FOM (mean of XY and θ) at/before t. */
    public  double getFomAt(double t) {
        for (int i = buffer.size() - 1; i >= 0; i--) {
            if (buffer.get(i).timestamp <= t) {
                Measurement m = buffer.get(i);
                return  (m.fomXY + m.fomTheta) / 2;
            }
        }
        return 0.0;
    }

    /** XY-only FOM at/before t. */
    public  double getFomXYAt(double t) {
        for (int i = buffer.size() - 1; i >= 0; i--) {
            if (buffer.get(i).timestamp <= t) {
                return buffer.get(i).fomXY;
            }
        }
        return 0.0;
    }

    /** θ-only FOM at/before t. */
    public  double getFomThetaAt(double t) {
        for (int i = buffer.size() - 1; i >= 0; i--) {
            if (buffer.get(i).timestamp <= t) {
                return buffer.get(i).fomTheta;
            }
        }
        return 0.0;
    }
}

