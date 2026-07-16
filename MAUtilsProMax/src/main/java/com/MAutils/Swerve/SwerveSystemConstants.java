
package com.MAutils.Swerve;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.MAutils.CanBus.CANBusID;
import com.MAutils.CanBus.SwerveModuleID;
import com.MAutils.Swerve.IOs.Gyro.Gyro;
import com.MAutils.Swerve.IOs.Gyro.GyroPiegon;
import com.MAutils.Swerve.IOs.Gyro.GyroReplay;
import com.MAutils.Swerve.IOs.Gyro.GyroReplayS2025;
import com.MAutils.Swerve.IOs.Gyro.GyroSim;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModule;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModuleReplay;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModuleReplayS2026;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModuleSim;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModuleTalonFX;
import com.MAutils.Swerve.Utils.ModuleLimits;
import com.MAutils.Swerve.Utils.PPHolonomicDriveController;
import com.MAutils.Utils.Constants;
import com.MAutils.Utils.Constants.SimulationType;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

public class SwerveSystemConstants {

        public enum GearRatio {
                L1(8.14, 1),
                L2(6.75, 2),
                L3(6.12, 3),
                L1Pluse(7.13, 7),
                L2Pluse(5.9, 8),
                L3Pluse(5.36, 9);

                public final double gearRatio;
                public final int ratioNumber;

                private GearRatio(double ratio, int ratioNumber) {
                        gearRatio = ratio;
                        this.ratioNumber = ratioNumber;
                }
        }

        public enum WheelType {
                BLACK_TREAD(1.426, 1.5),
                WCP_TREAD(1.0, 1.5); 

                public final double coFriction;
                public final double widthMM;

                private WheelType(double coFriction, double widthInches) {
                        this.coFriction = coFriction;
                        this.widthMM = Units.inchesToMeters(widthInches) * 1000; // Convert inches to mm
                }
        }

        public boolean OPTIMIZE = true;
        public double WIDTH = 0.566;
        public double LENGTH = 0.566;
        public double MOI = 1;
        public double RADIUS = Math.sqrt(
                        Math.pow(WIDTH, 2) + Math.pow(LENGTH, 2)) / 2.0;
        public double BUMPER_WIDTH = WIDTH + 0.16; 
        public double BUMPER_LENGTH = LENGTH + 0.16; 
        public double ROBOT_MASS = 62;
        public double TURNING_GEAR_RATIO = 150d / 7;
        public GearRatio DRIVE_GEAR_RATIO = GearRatio.L2;
        public double WHEEL_RADIUS = 0.0508;
        public double WHEEL_CIRCUMFERENCE = 2 * WHEEL_RADIUS * Math.PI;
        public DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1);
        public DCMotor TURNING_MOTOR = DCMotor.getFalcon500(1);
        public WheelType WHEEL_TYPE = WheelType.BLACK_TREAD;
        public final double MAX_STEER_VELOCITY_RADS = Units.rotationsToRadians(10.0);

        public SwerveModuleID[] MODULES_ID_ARRY;

        public InvertedValue[] DRIVES_INVERT = new InvertedValue[] {
                        InvertedValue.Clockwise_Positive,
                        InvertedValue.CounterClockwise_Positive,
                        InvertedValue.Clockwise_Positive,
                        InvertedValue.CounterClockwise_Positive
        };

        // Piegon
        public CANBusID PIEGEON_CAN_ID;

        // Module locations
        public Translation2d rearLeftLocation = new Translation2d(
                        -WIDTH / 2,
                        LENGTH / 2);

        public Translation2d frontLeftLocation = new Translation2d(
                        WIDTH / 2,
                        LENGTH / 2);

        public Translation2d rearRightLocation = new Translation2d(
                        -WIDTH / 2,
                        -LENGTH / 2);

        public Translation2d frontRightLocation = new Translation2d(
                        WIDTH / 2,
                        -LENGTH / 2);

        public Translation2d[] modulesLocationArry = new Translation2d[] { frontLeftLocation,
                        frontRightLocation, rearLeftLocation, rearRightLocation };

        public PPHolonomicDriveController realPPController = new PPHolonomicDriveController(new PIDConstants(3),
                        new PIDConstants(3));
        public PPHolonomicDriveController simPPController = new PPHolonomicDriveController(new PIDConstants(0),
                        new PIDConstants(0));;

        public PPHolonomicDriveController getPPController() {
                return RobotBase.isReal() ? realPPController : simPPController;
        }

        // Modules config
        public static int TELEOP_SLOT_CONFIG = 0;
        public static int AUTO_SLOT_CONFIG = 1;

        public GainConfig STEER_GainConfig = new GainConfig();

        public GainConfig DRIVE_TELEOP_GainConfig = new GainConfig();

        public GainConfig DRIVE_AUTO_GainConfig = new GainConfig();

        public double TURNING__CURRENT_LIMIT = 35;
        public boolean TURNING_ENABLE_CURRENT_LIMIT = true;

        public double DRIVE__SLIP_LIMIT = 35;
        public boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        // Kinamtics
        public double MAX_VELOCITY = 4.1;
        public double MAX_ACCELERATION = 5;
        public double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS * 0.9;// Radians
        public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        frontLeftLocation, frontRightLocation,
                        rearLeftLocation, rearRightLocation);

        public ModuleLimits DEFUALT = new ModuleLimits(MAX_VELOCITY, 18,//26.3
                        Units.degreesToRadians(700));

        public DriveTrainSimulationConfig DRIVE_TRAIN_SIMULATION_CONFIG = DriveTrainSimulationConfig
                        .Default()

                        .withGyro(COTS.ofPigeon2())
                        .withRobotMass(edu.wpi.first.units.Units.Kilogram.of(ROBOT_MASS))

                        .withSwerveModule(new SwerveModuleSimulationConfig(
                                        DRIVE_MOTOR,
                                        TURNING_MOTOR,
                                        DRIVE_GEAR_RATIO.gearRatio,
                                        TURNING_GEAR_RATIO,
                                        Volts.of(0.3),
                                        Volts.of(0.5),
                                        Meters.of(WHEEL_RADIUS),
                                        KilogramSquareMeters.of(0.05),
                                        WHEEL_TYPE.coFriction))
                        .withTrackLengthTrackWidth(edu.wpi.first.units.Units.Meter.of(LENGTH),
                                        edu.wpi.first.units.Units.Meter.of(WIDTH))
                        .withBumperSize(edu.wpi.first.units.Units.Meter.of(BUMPER_WIDTH),
                                        edu.wpi.first.units.Units.Meter.of(BUMPER_LENGTH));

        public SwerveDriveSimulation SWERVE_DRIVE_SIMULATION = new SwerveDriveSimulation(
                        DRIVE_TRAIN_SIMULATION_CONFIG, new Pose2d(2, 2, new Rotation2d()));

        // Module Limits
        public double ODOMETRY_UPDATE_RATE = 200;

        public SwerveSystemConstants() {

        }

        public SwerveSystemConstants withMotors(DCMotor driveMotor, DCMotor turningMotor,
                        SwerveModuleID[] modulesID, CANBusID piegonCANID) {
                this.PIEGEON_CAN_ID = piegonCANID;
                this.MODULES_ID_ARRY = modulesID;
                this.DRIVE_MOTOR = driveMotor;
                this.TURNING_MOTOR = turningMotor;
                return this;
        }

        public SwerveSystemConstants withGearRatio(GearRatio gearRatio) {
                this.DRIVE_GEAR_RATIO = gearRatio;
                return this;
        }

        public SwerveSystemConstants withPyshicalParameters(double with,
                        double length, double robotMass, WheelType wheelType, double moi) {
                this.WHEEL_TYPE = wheelType;
                this.WIDTH = with;
                this.LENGTH = length;
                this.ROBOT_MASS = robotMass;
                return this;
        }

        public SwerveSystemConstants withTurningTuning(GainConfig gainConfig) {
                this.STEER_GainConfig = gainConfig;
                return this;
        }

        public SwerveSystemConstants withDriveTuning(GainConfig gainConfig) {
                this.DRIVE_AUTO_GainConfig = gainConfig;
                this.DRIVE_TELEOP_GainConfig = gainConfig;
                return this;
        }

        public SwerveSystemConstants withDriveTuningTeleop(GainConfig gainConfig) {
                this.DRIVE_TELEOP_GainConfig = gainConfig;
                return this;
        }

        public SwerveSystemConstants withDriveAuto(GainConfig gainConfig) {
                this.DRIVE_AUTO_GainConfig = gainConfig;
                return this;
        }

        public SwerveSystemConstants withTurningCurrentLimit(double currentLimit, boolean enableCurrentLimit) {
                this.TURNING__CURRENT_LIMIT = currentLimit;
                this.TURNING_ENABLE_CURRENT_LIMIT = enableCurrentLimit;
                return this;
        }

        public SwerveSystemConstants withDriveCurrentLimit(double slipLimit, boolean enableCurrentLimit) {
                this.DRIVE__SLIP_LIMIT = slipLimit;
                this.DRIVE_ENABLE_CURRENT_LIMIT = enableCurrentLimit;
                return this;
        }

        public SwerveSystemConstants withMaxVelocityMaxAcceleration(double maxVelocity, double maxAcceleration) {
                this.MAX_VELOCITY = maxVelocity;
                this.MAX_ACCELERATION = maxAcceleration;
                this.MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS;
                return this;
        }

        public SwerveSystemConstants withOdometryUpdateRate(double odometryUpdateRate) {
                this.ODOMETRY_UPDATE_RATE = odometryUpdateRate;
                return this;
        }

        public SwerveSystemConstants withPPControllers(PPHolonomicDriveController realPPController,
                        PPHolonomicDriveController simPPController) {
                this.realPPController = realPPController;
                this.simPPController = simPPController;
                return this;
        }

        public SwerveSystemConstants withOptimize(boolean optimize) {
                this.OPTIMIZE = optimize;
                return this;
        }

        public static int getControlSlot() {
                return DriverStation.isAutonomous() ? AUTO_SLOT_CONFIG : TELEOP_SLOT_CONFIG;
        }

        //TODO the new need to be extrnal to the func, right know if you call this twich its will creat a new SwerveModule
        public SwerveModule[] getModules() {

                if (!Robot.isReal()) {
                        return Constants.SIMULATION_TYPE == SimulationType.SIM ? new SwerveModule[] {
                                        new SwerveModule("Front Left", this,
                                                        new SwerveModuleSim(this, 0,
                                                                        SWERVE_DRIVE_SIMULATION
                                                                                        .getModules()[0])),
                                        new SwerveModule("Front Right", this,
                                                        new SwerveModuleSim(this, 1,
                                                                        SWERVE_DRIVE_SIMULATION
                                                                                        .getModules()[1])),
                                        new SwerveModule("Rear Left", this,
                                                        new SwerveModuleSim(this, 2,
                                                                        SWERVE_DRIVE_SIMULATION
                                                                                        .getModules()[2])),
                                        new SwerveModule("Rear Right", this,
                                                        new SwerveModuleSim(this, 3, SWERVE_DRIVE_SIMULATION
                                                                        .getModules()[3])) }
                                        : new SwerveModule[] {
                                                        new SwerveModule("Front Left", this,
                                                                        new SwerveModuleReplayS2026("Front Left")),
                                                        new SwerveModule("Front Right", this,
                                                                        new SwerveModuleReplayS2026("Front Right")),
                                                        new SwerveModule("Rear Left", this,
                                                                        new SwerveModuleReplayS2026("Rear Left")),
                                                        new SwerveModule("Rear Right", this,
                                                                        new SwerveModuleReplayS2026("Rear Right")) };
                }

                return new SwerveModule[] {
                                new SwerveModule("Front Left", this, new SwerveModuleTalonFX(this, 0)),
                                new SwerveModule("Front Right", this, new SwerveModuleTalonFX(this, 1)),
                                new SwerveModule("Rear Left", this, new SwerveModuleTalonFX(this, 2)),
                                new SwerveModule("Rear Right", this, new SwerveModuleTalonFX(this, 3)) };
        }
        
//TODO smae commant as SwerveModule
        public Gyro getGyro() {

                if (!Robot.isReal()) {
                        return Constants.SIMULATION_TYPE == SimulationType.SIM ? new Gyro("Sim Gyro", new GyroSim(this))
                                        : new Gyro("Piegon Sim", new GyroReplayS2025());
                }

                return new Gyro("Piegon", new GyroPiegon(this));
        }

        public RobotConfig getRobotConfig() {
                return new RobotConfig(ROBOT_MASS, MOI,
                                new ModuleConfig(
                                                WHEEL_RADIUS, MAX_VELOCITY, WHEEL_TYPE.coFriction,
                                                DRIVE_MOTOR.withReduction(DRIVE_GEAR_RATIO.gearRatio),
                                                DRIVE__SLIP_LIMIT, 1),
                                modulesLocationArry);
        }

}
