
package com.MAutils.CanBus;

import com.ctre.phoenix6.CANBus;


/*
 * Represents a CAN bus ID along with its associated CAN bus.
 */
public class CANBusID {

    public final int id;
    public final CANBus bus;

    public CANBusID(int id, CANBus bus) {
        this.id = id;
        this.bus = bus;
    }


}
