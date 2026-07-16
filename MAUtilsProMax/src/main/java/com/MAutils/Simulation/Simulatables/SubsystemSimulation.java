
package com.MAutils.Simulation.Simulatables;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.MAutils.Simulation.Utils.Simulatable;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;
import com.MAutils.Utils.Constants;
import com.MAutils.Utils.ConvUtil;

public class SubsystemSimulation implements Simulatable {

    private PowerSystemConstants powerSystemConstants;

    public SubsystemSimulation(PowerSystemConstants powerSystemConstants) {
        this.powerSystemConstants = powerSystemConstants;
    }

    //TODO its better to move the subsystem and from the susbsytem get the constances
    @Override
    public void updateSimulation() {
        powerSystemConstants.masterSimState
                .setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

        powerSystemConstants.motorSim
                .setInputVoltage(powerSystemConstants.masterSimState.getMotorVoltage());
        powerSystemConstants.motorSim.update(Constants.LOOP_TIME);

        powerSystemConstants.masterSimState
                .setRawRotorPosition(powerSystemConstants.motorSim.getAngularPositionRotations()
                        * powerSystemConstants.GEAR);
        powerSystemConstants.masterSimState
                .setRotorVelocity(ConvUtil.RPMtoRPS(powerSystemConstants.motorSim.getAngularVelocityRPM())
                        * powerSystemConstants.GEAR);
    }

}
