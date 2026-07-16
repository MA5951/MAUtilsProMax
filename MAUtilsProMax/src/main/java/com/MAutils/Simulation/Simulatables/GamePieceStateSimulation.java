
package com.MAutils.Simulation.Simulatables;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.Simulation.SimulationManager;
import com.MAutils.Simulation.Utils.Simulatable;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class GamePieceStateSimulation implements Simulatable {

    public class GamePieceState {
        public final Supplier<Pose3d> gamePiecePoseSupplier;
        public final String stateName;
        public final Supplier<Boolean> stateEnterSupplier;

        public GamePieceState(String stateName, Supplier<Boolean> stateEnterSupplier,
                Supplier<Pose3d> gamePiecePoseSupplier) {
            this.stateName = stateName;
            this.gamePiecePoseSupplier = gamePiecePoseSupplier;
            this.stateEnterSupplier = stateEnterSupplier;
        }

    }
        //TODO need to add doc that the states need to be in the order of the robot flow
    public GamePieceStateSimulation(GamePieceState... states) {
        for (GamePieceState state : states) {
            gamePieceStates.add(state);
        }
        SimulationManager.registerSimulatable(this);
    }

    //TODO move the code in fail that it will make sense
    public final GamePieceState BLANK_STATE = new GamePieceState("BLANK_STATE", () -> false,
            () -> new Pose3d(-1, -1, -1, new Rotation3d()));

    private ArrayList<GamePieceState> gamePieceStates = new ArrayList<>();
    private GamePieceState currentState = BLANK_STATE;

    public GamePieceState getCurrentState() {
        return currentState;
    }

    @Override
    public void updateSimulation() {
        for (GamePieceState state : gamePieceStates) {
            if (state.stateEnterSupplier.get()) {
                currentState = state;
                break;
            }
        }

        MALog.log("Simulation/GamePiece/Robot/Current State Name/",
                currentState.stateName);
        MALog.log("Simulation/GamePiece/Robot/Current State Pose/", currentState.gamePiecePoseSupplier.get());
    }

}
