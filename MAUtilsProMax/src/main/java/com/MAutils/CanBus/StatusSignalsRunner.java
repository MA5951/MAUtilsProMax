
package com.MAutils.CanBus;

import com.MAutils.Logger.TelemetryLogger;
import com.ctre.phoenix6.BaseStatusSignal;


/*
 * Manages the registration and refreshing of status signals for different CAN buses.
 * 
 * Signals can be registered to either the Canivore bus or the Rio bus, and all registered signals can be refreshed periodically.
 */
public class StatusSignalsRunner {

    private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

    public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {

        for (BaseStatusSignal signal : signals) {
            TelemetryLogger.logSignal("Registerd Signal " + signal.getName() + " on " + (canivore ? "canivore" : "rio"));
        }
        if (canivore) {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
            System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
            System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
            canivoreSignals = newSignals;
        } else {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
            System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
            System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
            rioSignals = newSignals;
        }
    }

    public static void registerSignals(CANBusID canBusID, BaseStatusSignal... signals) { 
        registerSignals(canBusID.bus.getName() != "rio", signals);
    }

    public static void refreshAll() {
        if (canivoreSignals.length > 0) {
            BaseStatusSignal.refreshAll(canivoreSignals);
            TelemetryLogger.logSignal("Refreshing signals Canivore."); 
        }
        
        if (rioSignals.length > 0) {
            BaseStatusSignal.refreshAll(rioSignals);
            TelemetryLogger.logSignal("Refreshing signals Rio.");
        }
    }

}
