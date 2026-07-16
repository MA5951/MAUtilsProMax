
// package com.MAutils.Old;

// import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutoOption {

//     private Command autoCommand;
//     private String name;
//     private String pathPlannerName;
//     private Pose2d startPose;

//     public AutoOption(String optionName , String PathPlannerAutoName , Pose2d StartingPose) {
//         autoCommand = AutoBuilder.buildAuto(PathPlannerAutoName);
//         name = optionName;
//         pathPlannerName = PathPlannerAutoName;
//         startPose = StartingPose;
//     }

//     public AutoOption(String optionName,Command command ,Pose2d StartingPose,String previzPath) {
//         autoCommand = command;
//         name = optionName;
//         startPose = StartingPose;
//         pathPlannerName= previzPath;
//     }

//     public String getName() {
//         return name;
//     }

//     public String getPathPlannerAutoName() {
//         return pathPlannerName;
//     }

//     public Command getCommand() {
//         return autoCommand;
//     }

//     public Pose2d getStartPose() {
//         return startPose;
//     }
// }
