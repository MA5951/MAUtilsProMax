package com.MAutils.Swerve.Commands;

import com.MAutils.Swerve.SwerveSystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class WheelRadiusTest extends SequentialCommandGroup {

  private static final double WHEEL_RADIUS_RAMP_RATE = 0.5;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 2.0;
  private SwerveSystem swerveSystem;

  public WheelRadiusTest(SwerveSystem swerveSystem) {
    this.swerveSystem = swerveSystem;
    
    addCommands(wheelRadiusCharacterization());
  }

  private Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  swerveSystem.drive(new ChassisSpeeds(0.0, 0.0, speed));
                },
                swerveSystem)),

        Commands.sequence(
            Commands.waitSeconds(1.0),

            Commands.runOnce(
                () -> {
                  state.positions = getWheelRadiusCharacterizationPositions();
                  state.lastAngle = Rotation2d.fromDegrees(swerveSystem.getGyroData().yaw);
                  state.gyroDelta = 0.0;
                }),

            Commands.run(
                    () -> {
                      Rotation2d rotation = Rotation2d.fromDegrees(swerveSystem.getGyroData().yaw);
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                .finallyDo(
                    () -> {
                      double[] positions = getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * swerveSystem.getSwerveConstants().RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private double[] getWheelRadiusCharacterizationPositions() {
    double[] positions = new double[4];
    var modules = swerveSystem.getSwerveModules();
    
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getModuleData().drivePosition / swerveSystem.getSwerveConstants().WHEEL_RADIUS;
    }
    
    return positions;
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
