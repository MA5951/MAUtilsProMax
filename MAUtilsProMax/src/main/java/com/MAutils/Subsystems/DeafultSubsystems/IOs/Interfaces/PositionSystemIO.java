
package com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces;

import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;

public interface PositionSystemIO extends PowerSystemIO {

    double getSetPoint();

    double getError();

    boolean atPoint();

    void resetPosition(double pose);

    void setSystemConstants(PositionSystemConstants systemConstants, boolean burnMotor);

    void setPosition(double position);

    void setPosition(double position, double voltageFeedForward);

    void setPID(double kP, double kI, double kD);
    //TODO get closeLoopOutput
    //TODO add get Forwand and revers limits
    //TODO add get tourq

    //TODO add getSystemConstants

}
