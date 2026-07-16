
package com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces;

import com.MAutils.Subsystems.DeafultSubsystems.Constants.VelocitySystemConstants;

public interface VelocitySystemIO extends PowerSystemIO {

    double getSetPoint();

    double getError();

    boolean atPoint();

    void setSystemConstants(VelocitySystemConstants systemConstants);

    void setVelocity(double velocity);

    void setVelocity(double velocity, double feedForward);

    void setPID(double kP, double kI, double kD);

    //TODO add getaccelr
    //TODO get closeLoopOutput
    //TODO add getSystemConstants
}
