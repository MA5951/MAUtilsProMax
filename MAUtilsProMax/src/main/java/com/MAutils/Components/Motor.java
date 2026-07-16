package com.MAutils.Components;

import com.MAutils.CanBus.CANBusID;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;


/*
 * Represents a motor with its controller, type, name, inversion status, and CAN bus ID.
 */
public class Motor {

    public final TalonFX motorController;
    public final MotorType motorType;
    public final String name;
    public final InvertedValue invert;
    public final CANBusID canBusID;

    public Motor(CANBusID motorID, MotorType motorType, String name, InvertedValue invert) {
            this.canBusID = motorID;
            this.invert = invert;
            this.motorController = new TalonFX(motorID.id, motorID.bus);
            this.motorType = motorType;
            this.name = name;

        }

    public enum MotorType {
        KRAKEN,
        KRAKEN_FOC,
        FALCON,
        FALCON_FOC,
        NEO,
        BAG,
        CIM,
        MINI_NEO,
        MINI_CIM,
        RS775;

        public static DCMotor getDcMotor(MotorType type, int numMotors) {
            switch (type) {
                case KRAKEN:
                    return DCMotor.getKrakenX60(numMotors);
                case KRAKEN_FOC:
                    return DCMotor.getKrakenX60(numMotors);
                case FALCON:
                    return DCMotor.getFalcon500(numMotors);
                case FALCON_FOC:
                    return DCMotor.getFalcon500Foc(numMotors);
                case NEO:
                    return DCMotor.getNEO(numMotors);
                case BAG:
                    return DCMotor.getBag(numMotors);
                case CIM:
                    return DCMotor.getCIM(numMotors);
                case MINI_NEO:
                    return DCMotor.getNeo550(numMotors);
                case MINI_CIM:
                    return DCMotor.getMiniCIM(numMotors);
                case RS775:
                    return DCMotor.getVex775Pro(numMotors);
                default:
                    throw new IllegalArgumentException("Unknown motor type: " + type);
            }
        }
    }

}