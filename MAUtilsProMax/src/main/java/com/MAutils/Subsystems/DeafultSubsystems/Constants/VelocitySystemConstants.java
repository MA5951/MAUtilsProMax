package com.MAutils.Subsystems.DeafultSubsystems.Constants;

import com.MAutils.Components.Motor;
import com.MAutils.Utils.GainConfig;

import frc.robot.Robot;

public class VelocitySystemConstants extends DeafultSystemConstants<VelocitySystemConstants> {

    public final GainConfig REAL_GAINS;
    public final GainConfig SIM_GAINS;

    public final double MAX_VELOCITY;
    //TODO add max accle
     //TODO add ff
    public final double TOLERANCE ;
    public final double WHEEL_RADIUS;

    public VelocitySystemConstants(
            DeafultSystemConstants.Builder b,
            double maxVelocity,
            double tolerance,
            double wheelRadius,
            GainConfig realGains,
            GainConfig simGains) {
        super(b);
        this.MAX_VELOCITY = maxVelocity;
        this.TOLERANCE = tolerance;
        this.WHEEL_RADIUS = wheelRadius;

        this.REAL_GAINS = realGains;
        this.SIM_GAINS = simGains;
    }

    public GainConfig getGainConfig() {
        return Robot.isReal() ? REAL_GAINS : SIM_GAINS;
    }

    /*
     * =======================
     * Subclass Builder
     * =======================
     */

    /** Use a distinct name to avoid hiding the base static builder. */
    public static Builder newBuilder(String name, GainConfig realGains, Motor master, Motor... motors) {
        return new Builder(name, realGains, master, motors);
    }

    //TODO when you would use this class without init the other varubal? 
    //why you need a sec builder if you must init them 
    //TODO and if you stiil whant to use the builder tamplate you can extande tha main builder to this one and add a get func to the base one 
    //like the "folder" ctre have
    public static final class Builder {
        private final DeafultSystemConstants.Builder base;

        private double maxVelocity = 0;
        private double tolerance = 0;
        private double wheelRadius = 0;
       
       

        private GainConfig realGains = new GainConfig();
        private GainConfig simGains = new GainConfig().withKP(1);

        private Builder(String name, GainConfig realGains, Motor master, Motor... motors) {
            this.base = DeafultSystemConstants.builder(name, master, motors);
            this.realGains = realGains;
        }

        /* subclass options */
        public Builder maxVelocity(double maxVel) {
            this.maxVelocity = maxVel;
            return this;
        }

        public Builder tolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public Builder wheelRadius(double radius) {
            this.wheelRadius = radius;
            return this;
        }

        public Builder realGains(GainConfig gains) {
            this.realGains = gains;
            return this;
        }

        public Builder simGains(GainConfig gains) {
            this.simGains = gains;
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

        public Builder rampRate(double rampRate) {
            base.rampRate(rampRate);
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
            base.isBrake(brake); //TODO init by defult to be false
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

        public VelocitySystemConstants build() {
            return base.build(b -> new VelocitySystemConstants(
                    b,
                    maxVelocity,
                    tolerance,
                    wheelRadius,
                    realGains,
                    simGains));
        }
    }
}
