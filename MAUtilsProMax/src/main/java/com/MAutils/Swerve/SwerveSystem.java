
package com.MAutils.Swerve;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimationMA.OdometryObservation;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.PoseEstimation.SwerveDriveEstimator;
import com.MAutils.Simulation.Simulatables.SwerveSimulation;
import com.MAutils.Simulation.SimulationManager;
import com.MAutils.Swerve.IOs.Gyro.Gyro;
import com.MAutils.Swerve.IOs.Gyro.GyroIO.GyroData;
import com.MAutils.Swerve.IOs.PhoenixOdometryThread;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModule;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModuleTalonFX;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModuleIO.SwerveModuleData;
import com.MAutils.Swerve.Utils.ModuleLimits;
import com.MAutils.Swerve.Utils.SwerveSetPointGeneratorMA;
import com.MAutils.Swerve.Utils.SwerveSetpoint;
import com.MAutils.Swerve.Utils.SwerveState;
import com.MAutils.Utils.Constants;
import com.MAutils.Utils.DriverStationUtil;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SwerveSystem extends SubsystemBase {
    // private static SwerveSystem instance;

    private SwerveState currentState = new SwerveState("NONE");
    private SwerveDriveEstimator swerveDriveEstimator;

    private SwerveSetpoint currentSetpointMA = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            }); // TODO what the point?
    private Supplier<ModuleLimits> currentLimits; // TODO what the point? if you dont even have a set func for it, you
                                                  // should

    // TODO clean the code
    public static final Lock odometryLock = new ReentrantLock();
    private final SwerveSystemConstants swerveConstants;
    private final SwerveModule[] swerveModules;// FL FR RL RR
    private SwerveModuleData[] swerveModuleData = new SwerveModuleData[4];
    private final Gyro gyro;
    private ChassisSpeeds currentSpeeds;
    private SwerveSetpoint swerveSetpoint;
    private final SwerveSetPointGeneratorMA swerveSetPointGeneratorMA;
    private final SwerveModuleState[] currentStates = new SwerveModuleState[4];
    private final SwerveModulePosition[] currentPositions = new SwerveModulePosition[4];

    private final SwerveSetpointGenerator swerveSetpointGenerator; // TODO you must clean this code as soon as possibal
    private com.pathplanner.lib.util.swerve.SwerveSetpoint currSetpoint = new com.pathplanner.lib.util.swerve.SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            }, DriveFeedforwards.zeros(4));

    public SwerveSystem(SwerveSystemConstants swerveConstants) {
        super();
        this.swerveConstants = swerveConstants;

        swerveModules = swerveConstants.getModules();
        gyro = swerveConstants.getGyro();

        currentLimits = () -> swerveConstants.DEFUALT;

        swerveSetpointGenerator = new SwerveSetpointGenerator(swerveConstants.getRobotConfig(),
                swerveConstants.MAX_STEER_VELOCITY_RADS);

        if (!Robot.isReal()) {
            SimulationManager.registerSimulatable(new SwerveSimulation(swerveConstants));
        }

        swerveDriveEstimator = new SwerveDriveEstimator(swerveConstants, this);

        PhoenixOdometryThread.getInstance(swerveConstants).start();

        odometryLock.lock();
        gyro.update();
        for (var module : swerveModules) {
            module.update();
        }
        odometryLock.unlock();

        for (int i = 0; i < swerveModules.length; i++) {
            currentStates[i] = swerveModules[i].getState();
        }

        for (int i = 0; i < swerveModules.length; i++) {
            currentPositions[i] = swerveModules[i].getPosition();
        }

        // swerveSetpoint = new SwerveSetpoint(new ChassisSpeeds(0, 0, 0),
        // currentStates, DriveFeedforwards.zeros(4));

        swerveSetPointGeneratorMA = new SwerveSetPointGeneratorMA(swerveConstants.kinematics,
                swerveConstants.modulesLocationArry);
    }

   

    public double getTiltAngle() {
        return Math.sqrt(Math.pow(getGyroData().roll, 2) + Math.pow(getGyroData().pitch, 2));
    }

    public SwerveSystemConstants getSwerveConstants() {
        return swerveConstants;
    }

    public void setState(SwerveState state) {
        this.currentState = state;

    }

    public SwerveState getState() {
        return currentState;
    }

    public void isBrake(boolean isBrake) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDriveNutralMode(isBrake);
        }
    }

    public void periodic() {
        super.periodic();

        odometryLock.lock();
        gyro.update();
        for (var module : swerveModules) {
            module.update();
        }
        odometryLock.unlock();

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModuleData[i] = swerveModules[i].getModuleData();
            currentStates[i] = swerveModules[i].getState();
        }

        for (int i = 0; i < swerveModules.length; i++) {
            currentPositions[i] = swerveModules[i].getPosition();
        }

        currentSpeeds = swerveConstants.kinematics.toChassisSpeeds(currentStates);

        swerveDriveEstimator.updateOdometry();

        if (getTiltAngle() < 5) {
            PoseEstimationMA.getInstance().addOdometryObservation(new OdometryObservation(
                    Timer.getFPGATimestamp(), currentPositions, Optional.of(Rotation2d.fromDegrees(getGyroData().roll)),
                    Optional.of(Rotation2d.fromDegrees(getGyroData().pitch)),
                    Optional.of(Rotation2d.fromDegrees(getGyroData().yaw))),
                    swerveConstants.kinematics);
        }

        logSwerve();

    }

    // Public Methods
    public SwerveModuleState[] getCurrentStates() {
        return currentStates;
    }

    public Rotation3d getRobotRotation() {
        return new Rotation3d(getGyroData().roll, getGyroData().pitch, getGyroData().yaw);
    }

    public Rotation2d getRobotRotation2d() {
        return Rotation2d.fromDegrees(getGyroData().yaw);
    }

    public double getAbsYaw() {
        if (DriverStationUtil.getAlliance() == Alliance.Blue) {
            return getGyroData().yaw;
        }
        return getGyroData().yaw + 180;

    }

    public Supplier<Double> getAbsYawSupplier() {
        if (DriverStationUtil.getAlliance() == Alliance.Blue) {
            return () -> getGyroData().yaw;
        }
        return () -> getGyroData().yaw + 180;
    }

    public SwerveModulePosition[] getCurrentPositions() {
        return currentPositions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return currentSpeeds == null ? new ChassisSpeeds() : currentSpeeds;
    }

    public void drive(ChassisSpeeds speeds) {

        if (Double.isInfinite(speeds.vxMetersPerSecond) ||
                Double.isInfinite(speeds.vyMetersPerSecond) ||
                Double.isInfinite(speeds.omegaRadiansPerSecond) ||
                Double.isNaN(speeds.vxMetersPerSecond) ||
                Double.isNaN(speeds.vyMetersPerSecond) ||
                Double.isNaN(speeds.omegaRadiansPerSecond)) {
            speeds = new ChassisSpeeds();
        }

        // currSetpoint = swerveSetpointGenerator.generateSetpoint(currSetpoint, speeds,
        // Constants.LOOP_TIME);

        // MALog.logSwerveModuleStates("/Subsystems/Swerve/States/SetPoint",
        // currSetpoint.moduleStates());
        // runSwerveStates(currSetpoint.moduleStates());

        currentSetpointMA = swerveSetPointGeneratorMA.generateSetpoint(
                currentLimits.get(), currentSetpointMA, speeds, Constants.LOOP_TIME);

        MALog.logSwerveModuleStates("/Subsystems/Swerve/States/SetPoint",
                currentSetpointMA.moduleStates());
        runSwerveStates(currentSetpointMA.moduleStates());

        // MALog.logSwerveModuleStates("/Subsystems/Swerve/States/SetPoint",
        // swerveConstants.kinematics.toSwerveModuleStates(speeds));
        // runSwerveStates(swerveConstants.kinematics.toSwerveModuleStates(speeds));

    }

    public void runSwerveStates(SwerveModuleState[] states) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setSetPoint(states[i]);
        }

    }

    public GyroData getGyroData() {
        return gyro.getGyroData();
    }

    public Supplier<Double> getGyroYawSupplier() {
        return () -> getGyroData().yaw;
    }

    public SwerveModuleData[] getSwerveModuleData() {
        return swerveModuleData;
    }

    public Gyro getGyro() {
        return gyro;
    }

    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    private void logSwerve() {
        MALog.log("/Subsystems/Swerve/Chassis Speeds/Current", currentSpeeds);
        MALog.logSwerveModuleStates("/Subsystems/Swerve/States/Current", currentStates);
        MALog.log("/Subsystems/Swerve/States/Current State", currentState.getStateName());
        MALog.log("/Subsystems/Swerve/Abs Yaw", getAbsYawSupplier().get());
    }

}
