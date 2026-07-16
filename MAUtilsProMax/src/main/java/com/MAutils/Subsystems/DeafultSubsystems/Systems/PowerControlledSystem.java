
package com.MAutils.Subsystems.DeafultSubsystems.Systems;

import com.MAutils.RobotControl.State;
import com.MAutils.RobotControl.StateSubsystem;
import com.MAutils.Simulation.Simulatables.SubsystemSimulation;
import com.MAutils.Simulation.SimulationManager;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.PowerSystemIO;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.PowerControlled.PowerIOReal;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.PowerControlled.PowerIOReplay;
import com.MAutils.Utils.Constants;
import com.MAutils.Utils.Constants.SimulationType;

import frc.robot.Robot;

public abstract class PowerControlledSystem extends StateSubsystem { 

    protected PowerSystemIO systemIO;

    public PowerControlledSystem(PowerSystemConstants systemConstants, State... subsystemsStates) {
        super(systemConstants.SYSTEM_NAME, subsystemsStates);
        systemIO = new PowerIOReal(systemConstants); //TODO writ this as the else to if robot.isreal() you creat her onther instnace for nothing

        if (!Robot.isReal()) {
            if (Constants.SIMULATION_TYPE == SimulationType.SIM) {
                SimulationManager.registerSimulatable(new SubsystemSimulation(systemConstants));
            } else {
                systemIO = new PowerIOReplay(systemConstants);
            }
        }

        
    }

    public PowerControlledSystem(PowerSystemConstants systemConstants, PowerSystemIO simIO,  State... subsystemsStates) {
        super(systemConstants.SYSTEM_NAME, subsystemsStates);
        systemIO = new PowerIOReal(systemConstants); //TODO the smae commanet as befor

        if (!Robot.isReal()) {
            systemIO = simIO;
        }

    }

    public void setConstants(PowerSystemConstants systemConstants) {
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

    public boolean isMoving() {
        return systemIO.isMoving();
    }

    @Override
    public void periodic() {
        super.periodic();
        systemIO.updatePeriodic();
    }

}
