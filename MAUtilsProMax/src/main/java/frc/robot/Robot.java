// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultRobot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Util.Field;

public class Robot extends DeafultRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  // private final TalonFX motor;

  private final TalonFX shooterMotor;
  private final TalonFX kickerMotor;
  private final TalonFXConfiguration shooterConfig;
  private final TalonFXConfiguration kickerConfig;


  public Robot() {
    super();
    m_robotContainer = new RobotContainer();
    PoseEstimator.resetPose(Field.flipByAlliance(new Pose2d(3.586, 3.596, Rotation2d.fromDegrees(0))));
    frc.robot.Subsystems.Swerve.Swerve.getInstance();
   
    shooterMotor = new TalonFX(30);
    kickerMotor = new TalonFX(31);

    shooterConfig = new TalonFXConfiguration();
    kickerConfig = new TalonFXConfiguration();

    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    kickerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;    

    kickerMotor.getConfigurator().apply(kickerConfig);
    shooterMotor.getConfigurator().apply(shooterConfig);
    
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().setDefaultCommand(frc.robot.Subsystems.Swerve.Swerve.getInstance(),
        new frc.robot.Command.SwerveController());
  }

  @Override
  public void teleopPeriodic() {
    if (RobotContainer.getDriverController().getR1()) {
      shooterMotor.setVoltage(8);
      kickerMotor.setVoltage(8);
    } else {
      shooterMotor.setVoltage(0);
      kickerMotor.setVoltage(0);
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

}
