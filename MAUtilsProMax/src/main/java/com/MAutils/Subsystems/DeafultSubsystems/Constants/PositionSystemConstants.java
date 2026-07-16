package com.MAutils.Subsystems.DeafultSubsystems.Constants;

import com.MAutils.Components.Motor;
import com.MAutils.Utils.GainConfig;

import frc.robot.Robot;

public class PositionSystemConstants extends DeafultSystemConstants<PositionSystemConstants> {

    public final GainConfig realGainConfig;
    public final GainConfig simGainConfig;

    public final double MIN_POSE;
    public final double MAX_POSE;
    public final double START_POSE;
    public final double TOLERANCE;

    public final double CRUISE_VELOCITY;
    public final double ACCELERATION;
    public final double JERK;
    public final boolean IS_MOTION_MAGIC;

    public final double MASS;
    public final boolean IS_CONTINUOUS;


    

    /** Primary ctor used by the builder. */
    public PositionSystemConstants(
            DeafultSystemConstants.Builder b,
            double minPose,
            double maxPose,
            double startPose,
            double tolerance,
            GainConfig realGains,
            GainConfig simGains,
            boolean isMotionMagic,
            double cruiseVelocity,
            double acceleration,
            double jerk,
            double mass,
            boolean isContinuous) {
        super(b);
        this.MIN_POSE = minPose;
        this.MAX_POSE = maxPose;
        this.START_POSE = startPose;
        this.TOLERANCE = tolerance;
        this.IS_CONTINUOUS = isContinuous;

        this.realGainConfig = (realGains != null) ? realGains : new GainConfig();
        this.simGainConfig = (simGains != null) ? simGains : new GainConfig().withKP(1);

        this.IS_MOTION_MAGIC = isMotionMagic;
        this.CRUISE_VELOCITY = cruiseVelocity;
        this.ACCELERATION = acceleration;
        this.JERK = jerk;

        this.MASS = mass;
    }

    public GainConfig getGainConfig() {
        return Robot.isReal() ? realGainConfig : simGainConfig;
    }

    /*
     * =======================
     * Subclass Builder
     * =======================
     */

    /** Use a distinct name to avoid hiding the base static builder. */
    public static Builder newBuilder(String name, GainConfig realGains ,Motor master, Motor... motors) {
        return new Builder(name, realGains ,master, motors);
    }

    public static final class Builder {
        private final DeafultSystemConstants.Builder base;

        private double minPose = 0;
        private double maxPose = 0;
        private double startPose = 0;
        private double tolerance = 0;
        private boolean isContinuous = false;

        private GainConfig realGains = new GainConfig();
        private GainConfig simGains = new GainConfig().withKP(1);

        private boolean motionMagic = false;
        private double cruiseVelocity = 0; //TODO you can calculat this form the other constnace
        private double acceleration = 0; //TODO you can calculat this form the other constnace
        private double jerk = 0;

        private double mass = -1;

        private Builder(String name, GainConfig realGains ,Motor master, Motor... motors) {
            this.base = DeafultSystemConstants.builder(name, master, motors);
            this.realGains = realGains;
        }

        /* subclass options */
        public Builder range(double minPose, double maxPose) {
            this.minPose = minPose;
            this.maxPose = maxPose;
            return this;
        }

        public Builder startPose(double startPose) {
            this.startPose = startPose;
            return this;
        }

        public Builder tolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public Builder simGains(GainConfig gains) {
            this.simGains = gains;
            return this;
        }

        public Builder isContinuous(boolean continuous) {
            this.isContinuous = continuous;
            return this;
        }

        public Builder motionMagic(double cruiseVel, double accel, double jerk) {
            this.motionMagic = true;
            this.cruiseVelocity = cruiseVel;
            this.acceleration = accel;
            this.jerk = jerk;
            return this;
        }

        public Builder mass(double mass) {
            this.mass = mass;
            return this;
        }

        public Builder rampRate(double rampRate) {
            base.rampRate(rampRate);
            return this;
        }

        /* delegate base builder for fluent chaining */
        public Builder motorCurrentLimit(double amps) {
            base.motorCurrentLimit(amps);
            return this;
        }

        public Builder gear(double gear) {
            base.gear(gear);
            return this;
        }

        public Builder statorCurrentLimit(boolean enabled, double limit) {
            base.statorCurrentLimit(enabled, limit);
            return this;
        }

        public Builder peakVoltage(double forward, double reverse) {
            base.peakVoltage(forward, reverse);
            return this;
        }

        public Builder isBrake(boolean brake) {
            base.isBrake(brake);
            return this;
        }

        public Builder positionFactor(double factor) {
            base.positionFactor(factor);
            return this;
        }

        public Builder velocityFactor(double factor) {
            base.velocityFactor(factor);
            return this;
        }

        public Builder inertia(double inertia) {
            base.inertia(inertia);
            return this;
        }

        public Builder foc(boolean foc) {
            base.foc(foc);
            return this;
        }

        public Builder logPath(String path) {
            base.logPath(path);
            return this;
        }

        public PositionSystemConstants build() {
            return base.build(b -> new PositionSystemConstants(
                    b,
                    minPose, maxPose, startPose, tolerance,
                    realGains, simGains,
                    motionMagic, cruiseVelocity, acceleration, jerk,
                    mass, isContinuous));
        }
    }
}
