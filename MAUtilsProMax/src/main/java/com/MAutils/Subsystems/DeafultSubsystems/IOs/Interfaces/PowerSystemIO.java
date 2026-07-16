
package com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces;

import com.MAutils.Subsystems.DeafultSubsystems.Constants.DeafultSystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;

public interface PowerSystemIO {


    double getVelocity(); 

    double getPosition(); //TODO this also shouldt be her 

    double getCurrent(); //TODO add supplycurrent

    double getAppliedVolts(); 

    double getRawVelocity();//RPM //TODO raw is not in RPM... 

    double getRawPosition();//Degrees //TODO raw is not in Degrees...

    void setVoltage(double voltage); 

    void setBrakeMode(boolean isBrake); 

    void setSystemConstants(PowerSystemConstants systemConstants);

    void updatePeriodic();

    //TODO add set ramprate

    //TODO think about a func of setLimits

    default  public boolean isMoving() {
        return getRawVelocity() > DeafultSystemConstants.RPM_MOVING_THRESHOLD;
    }

    void restPosition(double position); //TODO way you need this in a power system?

    PowerSystemConstants getSystemConstants();

}
