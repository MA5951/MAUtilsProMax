
package com.MAutils.Controllers;

import com.MAutils.Logger.MALog;


//Add custom dead band TODO

/*
 * Interface representing a generic game controller with various buttons and axes.
 */

public interface MAController {

    public int getPort();

    public boolean getL1();

    public boolean getL2();

    public boolean getR1();

    public boolean getR2();

    public boolean getL3();

    public boolean getR3();

    public boolean getActionsUp();

    public boolean getActionsDown();

    public boolean getActionsLeft();

    public boolean getActionsRight();

    public boolean getDpadUp();

    public boolean getDpadDown();

    public boolean getDpadLeft();

    public boolean getDpadRight();

    public boolean getOptionsLeft();

    public boolean getOptionsRight();

    public boolean getMiddle();

    public double getRightTrigger(boolean withDeadbound, double scalar);

    public double getLeftTrigger(boolean withDeadbound, double scalar);

    public double getRightX(boolean withDeadbound, double scalar);

    public double getRightY(boolean withDeadbound, double scalar);

    public double getLeftX(boolean withDeadbound, double scalar);

    public double getLeftY(boolean withDeadbound, double scalar);

    public void setRumble(double power);

    default public double withDeadbound(double value) {
        return withDeadbound(value, 0.05);
    }

    default public double withDeadbound(double value, double deadbound) {
        return Math.abs(value) < deadbound ? 0 : value;
    }

    default public boolean inDeadbound() {
        return getRightX(true, 1) == 0 && getLeftX(true, 1) == 0 && getLeftY(true, 1) == 0;
    }

    default public boolean inDeadbound(double deadbound) {
        return getRightX(true, 1) == 0 && getLeftX(true, 1) == 0 && getLeftY(true, 1) == 0;
    }

    default public void log() {
        MALog.log("Controllers/" + getPort() + "/L1", getL1());
        MALog.log("Controllers/" + getPort() + "/L2", getL2());
        MALog.log("Controllers/" + getPort() + "/R1", getR1());
        MALog.log("Controllers/" + getPort() + "/R2", getR2());
        MALog.log("Controllers/" + getPort() + "/L3", getL3());
        MALog.log("Controllers/" + getPort() + "/R3", getR3());
        MALog.log("Controllers/" + getPort() + "/Actions/Up", getActionsUp());
        MALog.log("Controllers/" + getPort() + "/Actions/Down", getActionsDown());
        MALog.log("Controllers/" + getPort() + "/Actions/Left", getActionsLeft());
        MALog.log("Controllers/" + getPort() + "/Actions/Right", getActionsRight());
        MALog.log("Controllers/" + getPort() + "/Dpad/Up", getDpadUp());
        MALog.log("Controllers/" + getPort() + "/Dpad/Down", getDpadDown());
        MALog.log("Controllers/" + getPort() + "/Dpad/Left", getDpadLeft());
        MALog.log("Controllers/" + getPort() + "/Dpad/Right", getDpadRight());
        MALog.log("Controllers/" + getPort() + "/Options/Left", getOptionsLeft());
        MALog.log("Controllers/" + getPort() + "/Options/Right", getOptionsRight());
        MALog.log("Controllers/" + getPort() + "/Middle", getMiddle());
        MALog.log("Controllers/" + getPort() + "/RightTrigger", getRightTrigger(false, 1.0));
        MALog.log("Controllers/" + getPort() + "/LeftTrigger", getLeftTrigger(false, 1.0));
        MALog.log("Controllers/" + getPort() + "/RightX", getRightX(false, 1.0));
        MALog.log("Controllers/" + getPort() + "/RightY", getRightY(false, 1.0));
        MALog.log("Controllers/" + getPort() + "/LeftX", getLeftX(false, 1.0));
        MALog.log("Controllers/" + getPort() + "/LeftY", getLeftY(false, 1.0));
    }

}
