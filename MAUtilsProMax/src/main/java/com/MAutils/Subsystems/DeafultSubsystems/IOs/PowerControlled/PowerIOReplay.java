
package com.MAutils.Subsystems.DeafultSubsystems.IOs.PowerControlled;

import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.DeafultSystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.PowerSystemIO;
import com.MAutils.Utils.ConvUtil;

public class PowerIOReplay implements PowerSystemIO {

    private final PowerSystemConstants systemConstants;
    protected final String logPath;

    public PowerIOReplay(PowerSystemConstants systemConstants) {
        this.systemConstants = systemConstants;
        logPath = systemConstants.LOG_PATH == null ? "Subsystems/" + systemConstants.SYSTEM_NAME + "/IO" : systemConstants.LOG_PATH;
    }

    public double getVelocity() {
        return MALog.getReplayEntry(logPath + "/Velocity").getDouble(0);
    }

    public double getPosition() {
        return MALog.getReplayEntry(logPath + "/Position").getDouble(0);
    }

    public double getCurrent() {
        return MALog.getReplayEntry(logPath + "/Current").getDouble(0);
    }

    public double getAppliedVolts() {
        return MALog.getReplayEntry(logPath + "/Voltage").getDouble(0);
    }

    public void setVoltage(double voltage) {
    }

    public void setBrakeMode(boolean isBrake) {
    }

    @Override
    public void updatePeriodic() {
        MALog.log(logPath + "/Velocity", getVelocity());
        MALog.log(logPath + "/Voltage", getAppliedVolts());
        MALog.log(logPath + "/Current", getCurrent());
        MALog.log(logPath + "/Position", getPosition());

        MALog.log(logPath + "/Forward Limit", MALog.getReplayEntry(logPath + "/Forward Limit").getBoolean(false));
        MALog.log(logPath + "/Reverse Limit", MALog.getReplayEntry(logPath + "/Reverse Limit").getBoolean(false));

    }

    public boolean isMoving() {
        return ConvUtil.RPStoRPM(getVelocity() / systemConstants.VELOCITY_FACTOR) > DeafultSystemConstants.RPM_MOVING_THRESHOLD;
    }

    public void restPosition(double position) {
    }

    public PowerSystemConstants getSystemConstants() {
        return systemConstants;
    }

    @Override
    public double getRawVelocity() {
        return 0;
    }

    @Override
    public double getRawPosition() {
        return 0;
    }

    @Override
    public void setSystemConstants(PowerSystemConstants systemConstants) {
    }

    
}
