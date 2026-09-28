// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultRobot;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Util.Field;

public class Robot extends DeafultRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final TalonFX motor1, motor2;
  private TalonFXConfiguration config = new TalonFXConfiguration();

  private StatusSignal<Current> motor1CurrentSignal;
  private StatusSignal<Current> motor2CurrentSignal;


  public Robot() {
    super();
    m_robotContainer = new RobotContainer();
    PoseEstimator.resetPose(Field.flipByAlliance(new Pose2d(3.586,3.596, Rotation2d.fromDegrees(0))));
    frc.robot.Subsystems.Swerve.Swerve.getInstance();

    motor1 = new TalonFX(20);
    motor2 = new TalonFX(20);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    motor1CurrentSignal =motor1.getStatorCurrent();
    motor2CurrentSignal =motor2.getStatorCurrent();
    
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().setDefaultCommand(frc.robot.Subsystems.Swerve.Swerve.getInstance(), new frc.robot.Command.SwerveController());
  }

  @Override
  public void teleopPeriodic() {
    if(RobotContainer.getDriverController().getL1()) {
      motor1.setVoltage(7);
      motor2.setVoltage(7);
    } else {
      motor1.setVoltage(0);
      motor2.setVoltage(0);
    }


    MALog.log("Subsystems/Motor1/current", motor1CurrentSignal.getValueAsDouble());
    MALog.log("Subsystems/Motor2/current", motor2CurrentSignal.getValueAsDouble());

    motor1CurrentSignal.refresh();
    motor2CurrentSignal.refresh();
  }

  
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

 
}
