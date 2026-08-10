// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;

import com.MAutils.CanBus.CANBusID;
import com.MAutils.CanBus.SwerveModuleID;
import com.ctre.phoenix6.CANBus;

/** Add your docs here. */
public class PortMap {

    public class CAN_BUS {
        public static final CANBus RIO_BUS = new CANBus("rio");
        public static final CANBus CANIVORE_BUS = new CANBus("*");

    }

    public static class SwervePorts {

        private static final CANBusID LEFT_FRONT_ENCODER = new CANBusID(22, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID LEFT_FRONT_DRIVE = new CANBusID(8, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID LEFT_FRONT_TURNING = new CANBusID(5, CAN_BUS.CANIVORE_BUS);

        private static final CANBusID LEFT_BACK_ENCODER = new CANBusID(21, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID LEFT_BACK_DRIVE = new CANBusID(4, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID LEFT_BACK_TURNING = new CANBusID(9, CAN_BUS.CANIVORE_BUS);

        private static final CANBusID RIGHT_FRONT_ENCODER = new CANBusID(23, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID RIGHT_FRONT_DRIVE = new CANBusID(7, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID RIGHT_FRONT_TURNING = new CANBusID(6, CAN_BUS.CANIVORE_BUS);

        private static final CANBusID RIGHT_BACK_ENCODER = new CANBusID(24, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID RIGHT_BACK_DRIVE = new CANBusID(2, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID RIGHT_BACK_TURNING = new CANBusID(3, CAN_BUS.CANIVORE_BUS);
        public static final CANBusID PIGEON2 = new CANBusID(12, CAN_BUS.CANIVORE_BUS);

        public static final SwerveModuleID[] SWERVE_MODULE_IDS = {
                new SwerveModuleID(LEFT_FRONT_DRIVE, LEFT_FRONT_TURNING, LEFT_FRONT_ENCODER),
                new SwerveModuleID(RIGHT_FRONT_DRIVE, RIGHT_FRONT_TURNING, RIGHT_FRONT_ENCODER),
                new SwerveModuleID(LEFT_BACK_DRIVE, LEFT_BACK_TURNING, LEFT_BACK_ENCODER),
                new SwerveModuleID(RIGHT_BACK_DRIVE, RIGHT_BACK_TURNING, RIGHT_BACK_ENCODER)
        };

    }
}
