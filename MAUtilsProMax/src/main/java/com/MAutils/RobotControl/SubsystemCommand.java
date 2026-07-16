
package com.MAutils.RobotControl;

import edu.wpi.first.wpilibj2.command.Command;

/*
 * Abstract command class for subsystems that manage different modes of operation.
 */
public abstract class SubsystemCommand extends Command {

    protected StateSubsystem subsystem;

    public SubsystemCommand(StateSubsystem subsystem) {
        super();
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    public StateSubsystem getCommandSubsystem() {
        return subsystem;
    }

    public abstract void Automatic();

    public abstract void Manual();

    public abstract void CantMove();

    public void CanMove() {
        Automatic();
    }

    public void Auto() {
        Automatic();
    }

    public void Test() {

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (subsystem.CAN_MOVE() || subsystem.getSystemMode() == RobotControlConstants.SystemMode.MANUAL) {
            switch (RobotControlConstants.getRobotMode()) {
                case TELEOP:
                    switch (subsystem.getSystemMode()) {
                        case AUTOMATIC:
                            Automatic();
                            break;

                        default:
                            Manual();
                            break;
                    }
                    break;
                case AUTONOMOUS:
                    Auto();
                    break;
                case TEST:
                    Test();
                    break;
                default:
                    CantMove();
                    break;
            }
        } else {
            CantMove();
        }

    }

    @Override
    public void end(boolean interrupted) {
        CantMove();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
