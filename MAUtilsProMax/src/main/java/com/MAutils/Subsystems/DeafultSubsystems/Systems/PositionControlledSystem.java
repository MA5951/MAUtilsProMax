
package com.MAutils.Subsystems.DeafultSubsystems.Systems;

import com.MAutils.Logger.MALog;
import com.MAutils.RobotControl.State;
import com.MAutils.Simulation.Simulatables.SubsystemSimulation;
import com.MAutils.Simulation.SimulationManager;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.PositionSystemIO;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.PositionControlled.PositionIOReal;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.PositionControlled.PositionIOReplay;
import com.MAutils.Utils.Constants;
import com.MAutils.Utils.Constants.SimulationType;

import frc.robot.Robot;

public abstract class PositionControlledSystem extends PowerControlledSystem { 
    //TODO change it to extends the StateSubsystem and add func of get IO

    //and then you dont even need to cover the IO func you can acutly just change this class to a factory class, and add the getIO and upet to onther layer of subsystem 

    protected PositionSystemIO systemIO;

    public PositionControlledSystem(PositionSystemConstants systemConstants, State... subsystemsStates) {
        super(systemConstants.toPowerSystemConstants(), subsystemsStates);

        systemIO = new PositionIOReal(systemConstants); 
        //TODO this is a problam you creat a lot of io instace for nothing its beacuse this is an extra class in the midell

        if (!Robot.isReal()) {
            if (Constants.SIMULATION_TYPE == SimulationType.SIM) {
                SimulationManager.registerSimulatable(new SubsystemSimulation(systemIO.getSystemConstants()));
            } else {
                systemIO = new PositionIOReplay(systemConstants);
            }
        }

    }

    public PositionControlledSystem(PositionSystemConstants systemConstants, PositionSystemIO simIO,State... subsystemsStates) {
        super(systemConstants.toPowerSystemConstants(), subsystemsStates);
        systemIO = new PositionIOReal(systemConstants);

        if (!Robot.isReal()) {
            systemIO = simIO;
        }

    }

    public void setConstants(PositionSystemConstants systemConstants, boolean burnMotor) {
        systemIO.setSystemConstants(systemConstants, burnMotor);
    }

    public double getRawPosition() {
        return systemIO.getRawPosition();
    }

    public void resetPosition(double pose) {
        systemIO.resetPosition(pose);
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
        MALog.log(LOG_PATH + "Brake Mode", isBrake);
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

    public void setPosition(double position) {
        systemIO.setPosition(position);
    }

    public void setPosition(double position, double feedForward) {
        systemIO.setPosition(position, feedForward);
    }

    @Override
    public void periodic() {
        super.periodic();
        systemIO.updatePeriodic();
    }

}
