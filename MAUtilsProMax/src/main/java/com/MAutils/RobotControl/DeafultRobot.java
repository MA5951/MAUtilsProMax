
package com.MAutils.RobotControl;

import com.MAutils.Auto.AutoManager;
import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Logger.MALog;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Simulation.SimulationManager;
import com.MAutils.Utils.Constants;
import com.MAutils.Utils.Signal;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

/*
 * Default robot class that handles the main robot lifecycle methods.
 */
public class DeafultRobot extends TimedRobot {

    private static DeafultRobotContainer ROBOT_CONTAINER;
    private double startLoop = 0;
    private PowerDistribution pdh = new PowerDistribution();

    public DeafultRobot() {

    }

    @Override
    public void robotPeriodic() {
        StatusSignalsRunner.refreshAll();
        CommandScheduler.getInstance().run();
        PoseEstimator.update();

        log();

    }

    @Override
    public void disabledInit() {
        if (!Constants.COMPETITION_LOG) {
            MALog.stopLog();
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        Constants.cheackConstants();
        MALog.startLog(MALog.MALogMode.AUTO);

        if (!Robot.isReal()) {
            SimulationManager.autoInit();
        }

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (!Constants.COMPETITION_LOG) {
            MALog.startLog(MALog.MALogMode.TELEOP);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        MALog.stopLog();

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationInit() {
        SimulationManager.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        SimulationManager.updateSimulation();
    }

    public void log() {
        TelemetryLogger.logRio("Loop Time", startLoop - RobotController.getTime());
        TelemetryLogger.logRio("CPU Usage", RobotController.getCPUTemp() * 100);// TODO cheack how to do
        TelemetryLogger.logCanRio("TX", RobotController.getCANStatus().transmitErrorCount);
        TelemetryLogger.logCanRio("RX", RobotController.getCANStatus().receiveErrorCount);
        TelemetryLogger.logCanCanivore("TX", "");// TODO cheack how to do
        TelemetryLogger.logCanCanivore("RX", "");// TODO cheack how to do
        TelemetryLogger.logCanCanivore("Status","");// TODO cheack how to do
        TelemetryLogger.logPower("Voltage", pdh.getVoltage());
        TelemetryLogger.logPower("Amp", pdh.getTotalCurrent());

        TelemetryLogger.logDriverStation("Alliance", DriverStation.getAlliance().toString());
        TelemetryLogger.logDriverStation("Station", DriverStation.getLocation().toString());

        ROBOT_CONTAINER.driverController.log();
        ROBOT_CONTAINER.operatorController.log();

    }

}
