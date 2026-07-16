
package com.MAutils.Swerve.Commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModule;

public class FeedForwardTest extends SequentialCommandGroup {

  private static final double RAMP_RATE = 0.1;
  private List<Double> velocitySamples = new LinkedList<>(), voltageSamples = new LinkedList<>();
  private Timer timer = new Timer();
  private SwerveSystem swerveSystem;
  private double voltage, avgVelocity, wheelRadius;

  public FeedForwardTest(SwerveSystem swerveSystem) {
    this.swerveSystem = swerveSystem;
    wheelRadius = swerveSystem.getSwerveConstants().WHEEL_RADIUS;

    addCommands(
        new InstantCommand(this::resetLists),
        new InstantCommand(this::orianteModules),
        new WaitCommand(2),
        new InstantCommand(() -> timer.reset()),
        Commands.run(() -> {
          voltage = timer.get() * RAMP_RATE;
          setDriveVoltage(voltage);
          velocitySamples.add(getAvgVelocity());
          voltageSamples.add(voltage);
        }, swerveSystem)
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("Feed Forward Test Results");
                  System.out.println("kS: " + formatter.format(kS));
                  System.out.println("kV: " + formatter.format(kV));
                }));
  }

  private void resetLists() {
    velocitySamples.clear();
    voltageSamples.clear();

  }

  private void orianteModules() {
    for (SwerveModule module : swerveSystem.getSwerveModules()) {
      module.setSetPoint(new SwerveModuleState(0, new Rotation2d()));
    }
  }

  private void setDriveVoltage(double voltage) {
    for (SwerveModule module : swerveSystem.getSwerveModules()) {
      module.setDriveVoltage(voltage);
    }
  }

  private double getAvgVelocity() {
    for (SwerveModule module : swerveSystem.getSwerveModules()) {
      avgVelocity += module.getModuleData().driveVelocity / Math.PI * 2 * wheelRadius;
    }
    return avgVelocity / 4;
  }
}
