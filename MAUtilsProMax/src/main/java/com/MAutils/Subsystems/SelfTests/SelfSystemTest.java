
package com.MAutils.Subsystems.SelfTests;

import java.util.ArrayList;
import java.util.List;

import com.MAutils.Logger.MALog;
import com.MAutils.RobotControl.StateSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/*
 * Class for performing self-tests on subsystems.
 */
public class SelfSystemTest{

    private List<Test> testsList = new ArrayList<>();
    private final String logTable;
    private SequentialCommandGroup commandGroup;
    private final StateSubsystem subsystem;
    private boolean overallTestPassed;

    public SelfSystemTest(StateSubsystem  subsystem) {
        commandGroup = new SequentialCommandGroup();
        this.subsystem = subsystem;
        logTable = "Subsystem/" + subsystem.subsystemName + "/SelfTest/";
        commandGroup.addRequirements(subsystem);
    }

    public SelfSystemTest addTest(Test test) {
        testsList.add(test);
        return this;
    }

    private void runTest(Test test) {
        commandGroup.addCommands(

        new SequentialCommandGroup(
            new InstantCommand(() -> MALog.log(logTable + "Tests Status", "Runing test: " + test.testName))),
            new ConditionalCommand(
                new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(test.testTimeCap),
                    new InstantCommand(() -> MALog.log(logTable + "Tests Status", "Test finished passed: " + test.testName)),
                    new InstantCommand(() -> overallTestPassed = true)
                ),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !test.testCondition.getAsBoolean()),
                        new InstantCommand(() -> overallTestPassed = test.testCondition.getAsBoolean()),
                        new InstantCommand(() -> MALog.log(logTable + "Tests Status", "Test failed: " + test.testName))
                        
                    ),
                    new InstantCommand(test.testAction).repeatedly())
            )
                , 
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(test.testTimeCap),
                    new InstantCommand(() -> MALog.log(logTable + "Tests Status", "Test timed out: " + test.testName)),
                    new InstantCommand(() -> overallTestPassed = false)
                ),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(test.testCondition),
                        new InstantCommand(() -> overallTestPassed = test.testCondition.getAsBoolean()),
                        new InstantCommand(() -> MALog.log(logTable + "Tests Status", "Test passed: " + test.testName))
                        
                    ),
                    new InstantCommand(test.testAction).repeatedly())
            ), () -> test.TAG.equals("Test")),
            new InstantCommand(() -> MALog.log(logTable + "Tests Status", "Test Finished: " + test.testName))
        );
    }

    public Command createCommand() {
        overallTestPassed = false;
        commandGroup.addCommands(new InstantCommand(() -> MALog.log(logTable + "Self Test Status", "Starting Self Test For " + subsystem.subsystemName)));
        for (Test test : testsList) {
            runTest(test);
        }
        commandGroup.addCommands(new InstantCommand(() -> MALog.log(logTable + "Self Test Status", overallTestPassed ? "Passed" : "Failed" + ", Finished Self Test For " + subsystem.subsystemName)));
        return commandGroup;
    }

    public static Command fullTest(StateSubsystem... subsystem) {
        ParallelCommandGroup fullTestCommand = new ParallelCommandGroup();

        for (StateSubsystem system : subsystem) {
            fullTestCommand.addCommands(system.selfSystemTest.createCommand());
        }

        return fullTestCommand;
    }


}
