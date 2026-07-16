package com.MAutils.Controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;



/*
 * Adapter class to use an Xbox controller with the MAController interface.
 */
public class XboxMAController implements MAController {
    private final XboxController controller;
    private static final double TRIGGER_THRESHOLD = 0.03;

    public XboxMAController(int port) {
        this.controller = new XboxController(port);
        
    }

    @Override
    public int getPort() {
        return controller.getPort();
    }

    @Override
    public boolean getL1() {
        return controller.getLeftBumperButton();  
    }

    @Override
    public boolean getL2() {
        return controller.getLeftTriggerAxis() > TRIGGER_THRESHOLD;  
    }

    @Override
    public boolean getR1() {
        return controller.getRightBumperButton();  
    }

    @Override
    public boolean getR2() {
        return controller.getRightTriggerAxis() > TRIGGER_THRESHOLD;  
    }

    @Override
    public boolean getL3() {
        return controller.getLeftStickButton();  
    }

    @Override
    public boolean getR3() {
        return controller.getRightStickButton();  
    }

    @Override
    public boolean getActionsUp() {
        return controller.getYButton();  
    }

    @Override
    public boolean getActionsDown() {
        return controller.getAButton(); 
    }

    @Override
    public boolean getActionsLeft() {
        return controller.getXButton(); 
    }

    @Override
    public boolean getActionsRight() {
        return controller.getBButton();  
    }

    @Override
    public boolean getDpadUp() {
        return controller.getPOV() == 0; 
    }

    @Override
    public boolean getDpadDown() {
        return controller.getPOV() == 180; 
    }

    @Override
    public boolean getDpadLeft() {
        return controller.getPOV() == 270;  
    }

    @Override
    public boolean getDpadRight() {
        return controller.getPOV() == 90;  
    }

    @Override
    public boolean getOptionsLeft() {
        return controller.getBackButton();  
    }

    @Override
    public boolean getOptionsRight() {
        return controller.getStartButton();  
    }

    @Override
    public boolean getMiddle() {
        return false;
    }

    @Override
    public double getRightTrigger(boolean withDeadbound, double scalar) {
        if (withDeadbound) {
            return withDeadbound(controller.getRightTriggerAxis()) * scalar;
        }
        return controller.getRightTriggerAxis() * scalar;
    }

    @Override
    public double getLeftTrigger(boolean withDeadbound, double scalar) {
        if (withDeadbound) {
            return withDeadbound(controller.getLeftTriggerAxis()) * scalar;
        }
        return controller.getLeftTriggerAxis() * scalar;
    }

    @Override
    public double getRightX(boolean withDeadbound, double scalar) {
        if (withDeadbound) {
            return withDeadbound(controller.getRightX()) * scalar;
        }
        return controller.getRightX() * scalar;
    }

    @Override
    public double getRightY(boolean withDeadbound, double scalar) {
        if (withDeadbound) {
            return withDeadbound(controller.getRightY()) * scalar;
        }
        return controller.getRightY() * scalar;
    }

    @Override
    public double getLeftX(boolean withDeadbound, double scalar) {
        if (withDeadbound) {
            return withDeadbound(controller.getLeftX()) * scalar;
        }
        return controller.getLeftX() * scalar;
    }

    @Override
    public double getLeftY(boolean withDeadbound, double scalar) {
        if (withDeadbound) {
            return withDeadbound(controller.getLeftY()) * scalar;
        }
        return controller.getLeftY() * scalar;
    }

    @Override
    public double withDeadbound(double value) {
        return withDeadbound(value, 0.3);
    }

    @Override
    public void setRumble(double power) {
        controller.setRumble(RumbleType.kLeftRumble, power);
        controller.setRumble(RumbleType.kRightRumble, power);  
    }
}
