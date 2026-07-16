package com.MAutils.Subsystems.DeafultSubsystems.Constants;

import java.util.function.Function;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public abstract class DeafultSystemConstants<T> { //TODO ther isnt any point of it to be abstract class
        public static double RPM_MOVING_THRESHOLD = 0.5; // RPM

        public final Motor[] MOTORS; //TODO 
        public final Motor master;
        public final TalonFXSimState masterSimState;
        public final DCMotorSim motorSim;
        public final double GEAR;
        public final double STATOR_CURRENT_LIMIT;
        public final boolean CURRENT_LIMIT_ENABLED;
        public final double MOTOR_LIMIT_CURRENT; //rename to limit current.. its the limit of the system not the motor
        public final String LOG_PATH;
        public final boolean IS_BRAKE;
        public final double PEAK_FORWARD_VOLTAGE;
        public final double PEAK_REVERSE_VOLTAGE;
        public final boolean FOC;
        public final double INERTIA;
        public final double POSITION_FACTOR;
        public final double VELOCITY_FACTOR;
        public final String SYSTEM_NAME;
        public final double RAMP_RATE; 
        //TODO add supply curent limit
        public final LinearSystem<N2, N1, N2> systemID;

        /** Primary ctor used by subclasses and by the Builder-based ctor. */
        protected DeafultSystemConstants( 
                        String systemName,
                        Motor master,
                        double motorLimitCurrent,
                        double gear,
                        boolean currentLimitEnabled,
                        double statorCurrentLimit,
                        double peakForwardVoltage,
                        double peakReverseVoltage,
                        boolean isBrake,
                        double positionFactor,
                        double velocityFactor,
                        double inertia,
                        boolean foc,
                        String logPath,
                        double rampRate,
                        Motor... motors) {
                this.MOTORS = motors;
                this.master = master;
                this.SYSTEM_NAME = systemName;
                this.GEAR = gear;
                this.STATOR_CURRENT_LIMIT = statorCurrentLimit;
                this.CURRENT_LIMIT_ENABLED = currentLimitEnabled;
                this.MOTOR_LIMIT_CURRENT = motorLimitCurrent;
                this.LOG_PATH = logPath;
                this.PEAK_FORWARD_VOLTAGE = peakForwardVoltage;
                this.PEAK_REVERSE_VOLTAGE = peakReverseVoltage;
                this.FOC = foc;
                this.INERTIA = inertia;
                this.POSITION_FACTOR = positionFactor;
                this.VELOCITY_FACTOR = velocityFactor;
                this.IS_BRAKE = isBrake;
                this.RAMP_RATE = rampRate;

                masterSimState = master.motorController.getSimState();
                //TODO its better to change the sim object init to class func
                this.systemID = LinearSystemId.createDCMotorSystem(
                                MotorType.getDcMotor(master.motorType, 1 + MOTORS.length),
                                INERTIA, GEAR);

                motorSim = new DCMotorSim(
                                systemID,
                                MotorType.getDcMotor(master.motorType, 1 + MOTORS.length));
        }

        protected DeafultSystemConstants(Builder b) {
                this(
                                b.systemName,
                                b.master,
                                b.motorLimitCurrent,
                                b.gear,
                                b.currentLimitEnabled,
                                b.statorCurrentLimit,
                                b.peakForwardVoltage,
                                b.peakReverseVoltage,
                                b.isBrake,
                                b.positionFactor,
                                b.velocityFactor,
                                b.inertia,
                                b.foc,
                                b.logPath,
                                b.rampRate,
                                b.motors);
        }


        //TODO all the builder code should be extrnal to the class 
        //then you could use the power system as the base case that extand this as the tamplet       
        public PowerSystemConstants toPowerSystemConstants() { //TODO change to protected 
                return DeafultSystemConstants.from(this).build(PowerSystemConstants::new);
        }

        public static Builder builder(String systemName, Motor master, Motor... motors) { //TODO change to protected
                return new Builder(systemName, master, motors);
        }

        public static Builder from(DeafultSystemConstants<?> src) { //TODO change to protected
                return new Builder(src.SYSTEM_NAME, src.master, src.MOTORS)
                                .motorCurrentLimit(src.MOTOR_LIMIT_CURRENT)
                                .gear(src.GEAR)
                                .statorCurrentLimit(src.CURRENT_LIMIT_ENABLED, src.STATOR_CURRENT_LIMIT)
                                .peakVoltage(src.PEAK_FORWARD_VOLTAGE, src.PEAK_REVERSE_VOLTAGE)
                                .isBrake(src.IS_BRAKE)
                                .positionFactor(src.POSITION_FACTOR)
                                .velocityFactor(src.VELOCITY_FACTOR)
                                .inertia(src.INERTIA)
                                .foc(src.FOC)
                                .logPath(src.LOG_PATH);
        }

        public static final class Builder { //TODO change to protected
                // required
                private final String systemName;
                private final Motor master;
                private final Motor[] motors;

                // optionals (defaults mirror field defaults)
                private double motorLimitCurrent = 65;
                private double gear = 1;
                private boolean currentLimitEnabled = true;
                private double statorCurrentLimit = 60;
                private double peakForwardVoltage = 12;
                private double peakReverseVoltage = -12;
                private boolean isBrake = true;
                private double positionFactor = 1;
                private double velocityFactor = 60;
                private double inertia = 0.00001;
                private boolean foc = false;
                private String logPath = null; //TODO change to " "
                private double rampRate = 0;

                private Builder(String systemName, Motor master, Motor... motors) {
                        this.systemName = systemName;
                        this.master = master;
                        this.motors = motors;
                }

                public Builder motorCurrentLimit(double amps) {
                        this.motorLimitCurrent = amps;
                        return this;
                }

                public Builder gear(double gear) {
                        this.gear = gear;
                        return this;
                }

                public Builder statorCurrentLimit(boolean enabled, double limit) {
                        this.currentLimitEnabled = enabled;
                        this.statorCurrentLimit = limit;
                        return this;
                }

                public Builder peakVoltage(double forward, double reverse) {
                        this.peakForwardVoltage = forward;
                        this.peakReverseVoltage = reverse;
                        return this;
                }

                public Builder rampRate(double rate) {
                        this.rampRate = rate;
                        return this;
                }

                public Builder isBrake(boolean brake) {
                        this.isBrake = brake;
                        return this;
                }

                public Builder positionFactor(double factor) {
                        this.positionFactor = factor;
                        return this;
                }

                public Builder velocityFactor(double factor) {
                        this.velocityFactor = factor;
                        return this;
                }

                public Builder inertia(double inertia) {
                        this.inertia = inertia;
                        return this;
                }

                public Builder foc(boolean foc) {
                        this.foc = foc;
                        return this;
                }

                public Builder logPath(String path) {
                        this.logPath = path;
                        return this;
                }

                public <C extends DeafultSystemConstants<?>> C build(Function<Builder, C> ctor) {
                        return ctor.apply(this);
                }
        }
}
