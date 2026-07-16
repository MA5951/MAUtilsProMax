package com.MAutils.Controllers;

import edu.wpi.first.wpilibj.PS5Controller;


/*
 * Adapter class to use a PS5 controller with the MAController interface.
 */
public class PS5MAController implements MAController {
    private final PS5Controller controller;
    private XboxMAController xboxController;

    public PS5MAController(int port) {
        this.controller = new PS5Controller(port);
        this.xboxController = new XboxMAController(5 - port); 
    }

    @Override
    public int getPort() {
        return controller.getPort();
    }

    @Override
    public boolean getL1() {
        return controller.getL1Button();
    }

    @Override
    public boolean getL2() {
        return controller.getL2Button();
    }

    @Override
    public boolean getR1() {
        return controller.getR1Button();
    }

    @Override
    public boolean getR2() {
        return controller.getR2Button();
    }

    @Override
    public boolean getL3() {
        return controller.getL3Button();
    }

    @Override
    public boolean getR3() {
        return controller.getR3Button();
    }

    @Override
    public boolean getActionsUp() {
        return controller.getTriangleButton();
    }

    @Override
    public boolean getActionsDown() {
        return controller.getCrossButton();
    }

    @Override
    public boolean getActionsLeft() {
        return controller.getSquareButton();
    }

    @Override
    public boolean getActionsRight() {
        return controller.getCircleButton();
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
        return controller.getRawButton(9);
    }

    @Override
    public boolean getOptionsRight() {
        return controller.getOptionsButton();
    }

    @Override
    public boolean getMiddle() {
        return controller.getTouchpadButton();
    }

    @Override
    public double getRightTrigger(boolean withDeadbound, double scalar) {
        if (withDeadbound) {
            return withDeadbound(controller.getR2Axis()) * scalar;
        }
        return controller.getR2Axis() * scalar;
    }

    @Override
    public double getLeftTrigger(boolean withDeadbound, double scalar) {
        if (withDeadbound) {
            return withDeadbound(controller.getL2Axis()) * scalar;
        }
        return controller.getL2Axis() * scalar;
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
        return withDeadbound(value, 0.1);
    }

    @Override
    public void setRumble(double power) {
        xboxController.setRumble(power);
        xboxController.setRumble(power);
    }
}
