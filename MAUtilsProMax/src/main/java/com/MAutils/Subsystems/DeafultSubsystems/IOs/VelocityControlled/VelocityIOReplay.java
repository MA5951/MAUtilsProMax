
package com.MAutils.Subsystems.DeafultSubsystems.IOs.VelocityControlled;

import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.VelocitySystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.VelocitySystemIO;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.PowerControlled.PowerIOReplay;

public class VelocityIOReplay extends PowerIOReplay implements VelocitySystemIO {

    public VelocityIOReplay(VelocitySystemConstants systemName) {
        super(systemName.toPowerSystemConstants());
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

    public void setVelocity(double velocity) {
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
    public void setVelocity(double velocity, double feedForward) {
    }

    @Override
    public void setSystemConstants(VelocitySystemConstants systemConstants) {
    }

}
