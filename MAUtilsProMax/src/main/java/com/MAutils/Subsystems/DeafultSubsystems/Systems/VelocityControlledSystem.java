
package com.MAutils.Subsystems.DeafultSubsystems.Systems;

import com.MAutils.RobotControl.State;
import com.MAutils.Simulation.Simulatables.SubsystemSimulation;
import com.MAutils.Simulation.SimulationManager;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.VelocitySystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.VelocitySystemIO;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.VelocityControlled.VelocityIOReal;
import com.MAutils.Utils.Constants;
import com.MAutils.Utils.Constants.SimulationType;

import frc.robot.Robot;

public abstract class VelocityControlledSystem extends PowerControlledSystem {  
    //TODO change it to extends the StateSubsystem and add func of get IO

    protected VelocitySystemIO systemIO;

    public VelocityControlledSystem(VelocitySystemConstants systemConstants, State... subsystemsStates) {
        super(systemConstants.toPowerSystemConstants(), subsystemsStates);
        systemIO = new VelocityIOReal(systemConstants);

        if (!Robot.isReal()) {
            if (Constants.SIMULATION_TYPE == SimulationType.SIM) {
                SimulationManager.registerSimulatable(new SubsystemSimulation(systemIO.getSystemConstants()));
            } else {
                systemIO = new VelocityIOReal(systemConstants);
            }
        }

    }

    public VelocityControlledSystem(VelocitySystemConstants systemConstants, VelocitySystemIO simIO,
            State... subsystemsStates) {
        super(systemConstants.toPowerSystemConstants(), subsystemsStates);
        systemIO = new VelocityIOReal(systemConstants);

        if (!Robot.isReal()) {
            systemIO = simIO;
        }

    }

    public void setConstants(VelocitySystemConstants systemConstants) {
        systemIO.setSystemConstants(systemConstants);
    }

    public double getRawPosition() {
        return systemIO.getRawPosition();
    }

    public double getRawVelocity() {
        return systemIO.getRawVelocity();
    }

    public double getAppliedVolts() {
        return systemIO.getAppliedVolts();
    }

    public double getCurrent() {
        return systemIO.getCurrent();
    }

    public double getPosition() {
        return systemIO.getPosition();
    }

    public double getVelocity() {
        return systemIO.getVelocity();
    }

    public void setVoltage(double voltage) {
        systemIO.setVoltage(voltage);
    }

    public void setBrakeMode(boolean isBrake) {
        systemIO.setBrakeMode(isBrake);
    }

    public double getSetPoint() {
        return systemIO.getSetPoint();
    }

    public double getError() {
        return systemIO.getError();
    }

    public boolean atPoint() {
        return systemIO.atPoint();
    }

    public void setVelocity(double velocity) {
        systemIO.setVelocity(velocity);
    }

    public void setVelocity(double velocity, double feedForward) {
        systemIO.setVelocity(velocity, feedForward);
    }

    @Override
    public void periodic() {
        super.periodic();
        systemIO.updatePeriodic();
    }

}
