
package com.MAutils.Simulation.Utils;


public interface Simulatable {

    default void simulationInit() {}

    default void updateSimulation() {}

    default void autoInit() {}

}
