
package com.MAutils.Components;

import java.util.function.Supplier;

import frc.robot.Robot;


/*
 * A wrapper for a boolean sensor that allows for easy simulation and real-world usage.
 */
public class BooleanSensorWrapper {
    //TODO if not use delete or put daprecated

    private Supplier<Boolean> sensorSupplier;
    private Boolean value;
    private boolean usingSupplier;


    public BooleanSensorWrapper(Supplier<Boolean> sensorSupplier, Supplier<Boolean> sensorSimSupplier) {
        this.sensorSupplier = sensorSupplier;
        usingSupplier = true;

        if (!Robot.isReal()) {
            sensorSupplier = sensorSimSupplier;
        }
    }

    public void setValue(Boolean value) {
        if (!Robot.isReal()) {
            this.value = value;
            usingSupplier = false;
        } else {
            throw new UnsupportedOperationException("Cannot set value in real mode.");
        }
    }

    public Boolean getValue() {
        return usingSupplier ? sensorSupplier.get() : value;
    }

    public void useSupplier() {
        usingSupplier = true;
    }


}
