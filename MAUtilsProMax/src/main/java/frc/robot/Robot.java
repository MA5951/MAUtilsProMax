// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Components.MACam;
import com.MAutils.Logger.MALog;
import com.MAutils.RobotControl.DeafultRobot;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.MAcam.MAcam;

public class Robot extends DeafultRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final StrictFollower shooterControl, transferControl;

  private final MAcam feederMacam, intakeMacam, transferMacam;

  private final TalonFX shooterMaster, shooterSlave, hoodMotor, transferMaster, transferSlave;

  private final TalonFXConfiguration masterConfig, slaveConfig, hoodConfig, transferMasterConfig, transferSlaveConfig;

  private final StatusSignal<AngularVelocity> shooterVelocity, hoodVelocity;
  private final StatusSignal<Voltage> shooterVoltage, hoodVoltage;

  private final StatusSignal<Current> transferMasterCurrent, transferSlaveCurrent, hoodCurrent, shooterSlaveCurrent,
      shooterMasterCurrent;

  public Robot() {
    super();
    m_robotContainer = new RobotContainer();

    shooterMaster = new TalonFX(33);
    shooterSlave = new TalonFX(34);
    hoodMotor = new TalonFX(37);
    transferMaster = new TalonFX(42);
    transferSlave = new TalonFX(43);

    masterConfig = new TalonFXConfiguration();
    slaveConfig = new TalonFXConfiguration();
    hoodConfig = new TalonFXConfiguration();
    transferMasterConfig = new TalonFXConfiguration();
    transferSlaveConfig = new TalonFXConfiguration();

    feederMacam = new MAcam(7);
    intakeMacam = new MAcam(8);
    transferMacam = new MAcam(9);

    shooterControl = new StrictFollower(shooterMaster.getDeviceID());
    transferControl = new StrictFollower(transferMaster.getDeviceID());

    masterConfig.Slot0.kP = 0.0;
    masterConfig.Slot0.kI = 0.0;
    masterConfig.Slot0.kD = 0.0;
    masterConfig.Slot0.kV = 0.0;
    masterConfig.Slot0.kS = 0.0;

    hoodConfig.Slot0.kP = 0.0;
    hoodConfig.Slot0.kI = 0.0;
    hoodConfig.Slot0.kD = 0.0;
    hoodConfig.Slot0.kV = 0.0;
    hoodConfig.Slot0.kS = 0.0;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    slaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    slaveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    transferMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    transferMasterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    transferSlaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    transferSlaveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterMaster.getConfigurator().apply(masterConfig);
    shooterSlave.getConfigurator().apply(slaveConfig);
    hoodMotor.getConfigurator().apply(hoodConfig);
    transferMaster.getConfigurator().apply(transferMasterConfig);
    transferSlave.getConfigurator().apply(transferSlaveConfig);

    shooterVelocity = shooterMaster.getVelocity();
    hoodVelocity = hoodMotor.getVelocity();
    shooterVoltage = shooterMaster.getMotorVoltage();
    hoodVoltage = hoodMotor.getMotorVoltage();
    shooterMasterCurrent = shooterMaster.getStatorCurrent();
    shooterSlaveCurrent = shooterSlave.getStatorCurrent();
    hoodCurrent = hoodMotor.getStatorCurrent();
    transferMasterCurrent = transferMaster.getStatorCurrent();
    transferSlaveCurrent = transferSlave.getStatorCurrent();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (RobotContainer.getDriverController().getL1()) {
      // shooterMaster.setControl(new VelocityVoltage(5000 / 60));
      shooterMaster.setVoltage(8);
      shooterSlave.setControl(shooterControl);
      // hoodMotor.setControl(new VelocityVoltage(1000 / 60));
      hoodMotor.setVoltage(8);
      transferMaster.setVoltage(6);
      transferSlave.setControl(transferControl);
    } else {
      // shooterMaster.setControl(new VelocityVoltage(0));
      shooterMaster.setVoltage(0);
      shooterSlave.setControl(shooterControl);
      // hoodMotor.setControl(new VelocityVoltage(0));
      hoodMotor.setVoltage(0);
      transferMaster.setVoltage(0);
      transferSlave.setControl(transferControl);
    }

    shooterVelocity.refresh();
    hoodVelocity.refresh();
    shooterVoltage.refresh();
    hoodVoltage.refresh();
    shooterMasterCurrent.refresh();
    shooterSlaveCurrent.refresh();
    hoodCurrent.refresh();
    transferMasterCurrent.refresh();
    transferSlaveCurrent.refresh();

    MALog.log("Shooter/Shooter Velocity", shooterVelocity.getValueAsDouble() / 60);
    MALog.log("Hood/Hood Velocity", hoodVelocity.getValueAsDouble() / 60);
    MALog.log("Shooter/Shooter Voltage", shooterVoltage.getValueAsDouble());
    MALog.log("Hood/Hood Voltage", hoodVoltage.getValueAsDouble());
    MALog.log("Shooter/Shooter Current", shooterMasterCurrent.getValueAsDouble());
    MALog.log("Shooter/Shooter Current", shooterSlaveCurrent.getValueAsDouble());
    MALog.log("Hood/Hood Current", hoodCurrent.getValueAsDouble());
    MALog.log("Transfer/Transfer Current", transferMasterCurrent.getValueAsDouble());
    MALog.log("Transfer/Transfer Current", transferSlaveCurrent.getValueAsDouble());
    MALog.log("Feeder/Feeder MAcam Distance", feederMacam.getDistanceMM());
    MALog.log("Intake/Intake MAcam Distance", intakeMacam.getDistanceMM());
    MALog.log("Transfer/Transfer MAcam Distance", transferMacam.getDistanceMM());
  }

  @Override
  public void teleopExit() {
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
}
