
package com.MAutils.Subsystems.DeafultSubsystems.IOs.PositionControlled;

import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.PositionSystemIO;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.PowerControlled.PowerIOReplay;

public class PositionIOReplay extends PowerIOReplay implements PositionSystemIO {

    public PositionIOReplay(PositionSystemConstants systemConstants) {
        super(systemConstants.toPowerSystemConstants());

    }

    public double getSetPoint() {
        return MALog.getReplayEntry(logPath + "/Set Point").getDouble(0);
    }

    public double getError() {
        return MALog.getReplayEntry(logPath + "/Error").getDouble(0);
    }

    public boolean atPoint() {
        return MALog.getReplayEntry(logPath + "/At Point").getBoolean(false);
    }

    public void setPosition(double position) {
    }

    public void setPosition(double position, double voltageFeedForward) {
    }

    public void setPID(double kP, double kI, double kD) {
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();

        MALog.log(logPath + "/Set Point", getSetPoint());
        MALog.log(logPath + "/Error", getError());
        MALog.log(logPath + "/At Point", atPoint());

       
    }

    @Override
    public void setSystemConstants(PositionSystemConstants systemConstants, boolean burnMotor) {
    }

    @Override
    public void resetPosition(double pose) {
        //TODO ? 
        // throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

}
