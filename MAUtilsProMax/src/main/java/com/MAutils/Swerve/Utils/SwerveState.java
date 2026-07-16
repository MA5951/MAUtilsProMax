
package com.MAutils.Swerve.Utils;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveState {

    private ChassisSpeeds stateSpeeds;
    private String stateName;
    private Supplier<ChassisSpeeds> xySupplier, omegaSupplier;
    private Runnable onStateEnter = () -> {}, onStateRuning = () -> {}, updatRunnableXY = () -> {}, updatRunnableOmega = () -> {};

//TODO why you use new chassis speed all the time insted of update stateSpeeds
//TODO and its make more sense to give the swervestate the controller throw the constructor
    public SwerveState(String name) {
        this.stateName = name;
        this.stateSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    public SwerveState withOnStateEnter(Runnable onStateEnter) {
        this.onStateEnter = onStateEnter;
        return this;
    }

    public SwerveState withOnStateRuning(Runnable onStateRuning) {
        this.onStateRuning = onStateRuning;
        return this;
    }

    public Runnable getOnStateEnter() {
        return onStateEnter;
    }
    
    public Runnable getOnStateRuning() {
        return onStateRuning;
    }

    public SwerveState withXY(SwerveController controller) {
        updatRunnableXY = controller::updateControllers;
        xySupplier = () -> controller.getSpeeds();
        return this;
    }

    public SwerveState withXY(SwerveController controller, SwerveController controller1, Supplier<Double> scale1) {
        updatRunnableXY = () -> {
            controller.updateControllers();
            controller1.updateControllers();
        };
        xySupplier = () -> controller.getSpeeds().plus(controller1.getSpeeds().times(scale1.get()));
        return this;
    }

    public SwerveState withOmega(SwerveController controller) {
        updatRunnableOmega = controller::updateControllers;
        omegaSupplier = () -> controller.getSpeeds();
        return this;
    }

    public SwerveState withSpeeds(SwerveController controller) {
        updatRunnableXY = controller::updateControllers;
        xySupplier = () -> controller.getSpeeds();
        omegaSupplier = () -> controller.getSpeeds();
        return this;
    }

    public SwerveState withSpeeds(ChassisSpeeds speeds) {
        xySupplier = () -> speeds;
        omegaSupplier = () -> speeds;
        return this;
    }

    public SwerveState withXY(double x, double y) {
        xySupplier = () -> new ChassisSpeeds(x, y, 0);
        return this;
    }

    public SwerveState withOmega(double omega) {
        omegaSupplier = () -> new ChassisSpeeds(0, 0, omega);
        return this;
    }

    public SwerveState withXY(Supplier<Double> x, Supplier<Double> y) {
        xySupplier = () -> new ChassisSpeeds(x.get(), y.get(), 0);
        return this;
    }

    public SwerveState withOmega(Supplier<Double> omega) {
        omegaSupplier = () -> new ChassisSpeeds(0, 0, omega.get());
        return this;
    }

    public SwerveState setX(double x) {
        xySupplier = () -> new ChassisSpeeds(x, 0, 0);
        return this;
    }

    public SwerveState setY(double y) {
        
        xySupplier = () -> new ChassisSpeeds(0, y, 0);
        return this;
    }

    public ChassisSpeeds getSpeeds() {
        updatRunnableXY.run();//TODO why not just call the updateControllers of the swervecontroller in the getSpeeds func and delte the use of the runbal
        //right know its what happning just with a runbal in the way 
        updatRunnableOmega.run();

        stateSpeeds = xySupplier.get();
        stateSpeeds.omegaRadiansPerSecond = omegaSupplier.get().omegaRadiansPerSecond;
        return stateSpeeds;
    }

    public String getStateName() {
        return stateName;
    }

}
