
package com.MAutils.Simulation.Simulatables;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.MAutils.Logger.MALog;
import com.MAutils.Simulation.Utils.Simulatable;
import com.MAutils.Swerve.SwerveSystemConstants;

public class SwerveSimulation implements Simulatable {

    private SwerveDriveSimulation swerveDriveSimulation;
    //TODO hear also is better to give the swervesubsystem but its les critacle because we olny got one swerev
    public SwerveSimulation(SwerveSystemConstants swerveSystemConstants) {
        swerveDriveSimulation = swerveSystemConstants.SWERVE_DRIVE_SIMULATION;
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
    }

    @Override
    public void updateSimulation() {
        SimulatedArena.getInstance().simulationPeriodic(); // TODO why its her and not in the SimulationManager? 
        MALog.log("Simulation/Simulation Pose", swerveDriveSimulation.getSimulatedDriveTrainPose());
    }

}
