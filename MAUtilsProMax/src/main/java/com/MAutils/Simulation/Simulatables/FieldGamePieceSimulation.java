
package com.MAutils.Simulation.Simulatables;

import org.ironmaple.simulation.SimulatedArena;

import com.MAutils.Logger.MALog;
import com.MAutils.Simulation.Utils.Simulatable;

public class FieldGamePieceSimulation implements Simulatable{

    private String[] gamePieces;
    private boolean resetForAuto;

    public FieldGamePieceSimulation(boolean resetForAuto, String... gamePieces) {
        this.gamePieces = gamePieces;
        this.resetForAuto = resetForAuto;
    }

    @Override
    public void autoInit() {
        SimulatedArena.getInstance().clearGamePieces();
        if (resetForAuto) {
            SimulatedArena.getInstance().resetFieldForAuto();
        }
    }

    @Override
    public void simulationInit() {
        autoInit();
    }


    @Override
    public void updateSimulation() {
        for (String type : gamePieces) {
            MALog.log("Simulation/GamePices/" + type, SimulatedArena.getInstance().getGamePiecesArrayByType(type));
        }
    }

}
