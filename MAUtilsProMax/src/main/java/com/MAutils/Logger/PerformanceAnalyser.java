
package com.MAutils.Logger;

import java.util.function.Supplier;

import com.MAutils.RobotControl.MRobotState;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;
import com.MAutils.Swerve.SwerveSystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;


/*
 * Used to analyse the performance of the robot during operation.
 */
public class PerformanceAnalyser {
    //Note: Redundent feature, auto is being timed here as well as in automanager, consider removing one of them
    private static final double VELOCITY_TOLERANC = 0.1; 
    private static final double ACCLERATION_TOLERANC = 1; 

    private static class MeasureEntry {
        private String name;
        private double precent;
        public MeasureEntry(String name, double precent) {
            this.name = name;
            this.precent = precent;
        }

        public String getName() { //TODO if not use delete 
            return name;
        }

        public double getPrecent() { //TODO if not use delete 
            return precent;
        }
    }

    private static class acceleration {
        private double acceleration;
        private double accelerationTime;
        public acceleration(double acceleration, double accelerationTime) {
            this.acceleration = acceleration;
            this.accelerationTime = accelerationTime;
        }

        public double getAcceleration() { 
            return acceleration;
        }

        public double getAccelerationTime() {
            return accelerationTime;
        }
    }

    private static class velocity {
        private double velocity;
        private double velocityTime;
        public velocity(double velocity, double velocityTime) {
            this.velocity = velocity;
            this.velocityTime = velocityTime;
        }

        public double getVelocity() {
            return velocity;
        }

        public double getVelocityTime() {
            return velocityTime;
        }
    }

    //TODO is probalbly Better to the use delta time and getFPGATimestamp
    private static final Timer timeInCurrentStatetimer = new Timer();
    private static final Timer timeBetweenStatesTimer = new Timer();
    private static final Timer timeInMaxSpeed = new Timer();
    private static final Timer accelerationTimer =  new Timer();
    private static final Timer timerTimeInAcceleration = new Timer();

    private static MRobotState lastRobotState = null;
    private static MRobotState firstRobotState = null; 
    private static MRobotState secondRobotState = null; 

    private static Supplier<MRobotState> currentRobotState;

    private static int cycleNum = 0; 

    private static double maxVelocity = 0;
    private static double maxVelocityTime = 0;
    private static double sumOfCurrents = 0;
    private static double sumOfVoltage = 0;
    private static double minCycleTime = 999;  
    private static double maxCycleTime = 0;
    private static double allCycles = 0;
    private static double timeInCurrentState = 0;
    private static double timeBetweenState = 0;
    private static double lastVelocity = 0;
    private static double lastTime = 0;
    private static double currentAcceleration = 0;
    private static double maxAcceleration = 0;
    private static double timeInAcceleration = 0;

    private static Supplier<Double> currentVelocity;

    private static double[] systemCurrents;
    private static double[] systemVoltage;
    private static PowerControlledSystem[] subsystems;
    private static MeasureEntry[] currentPrecents;
    private static MeasureEntry[] voltagePrecents;


    private static final PowerDistribution PDH = new PowerDistribution();

    private static boolean logCycleTimes = false;
    private static boolean logTimeCurrentState = false;
    private static boolean logMaxVelocity = false;
    private static boolean logCurrent = false ;
    private static boolean logVoltage = false;
    private static boolean logAcceleration = false;
    private static boolean cycleDone = false;

    public static Supplier<Boolean> firstCondition;
    public static Supplier<Boolean> endCondition;


    private static double getTimeInCurrentState() {
        if (lastRobotState == null) {
            timeInCurrentStatetimer.start();
            lastRobotState = currentRobotState.get();
        }
        timeInCurrentStatetimer.start(); 

        if (currentRobotState.get() != lastRobotState) {
            lastRobotState = currentRobotState.get(); //TODO you should call this always
            timeInCurrentState = timeInCurrentStatetimer.get(); 
            timeInCurrentStatetimer.reset();
            return timeInCurrentState;
        } 
            
        return -1;
    }

    private static double getCycleTime() { 

        if (currentRobotState.get() == firstRobotState && firstCondition.get()) {
            timeBetweenState = -1; // TODO just init it to - 1
            timeBetweenStatesTimer.start();
        } else if (currentRobotState.get() == secondRobotState && endCondition.get() && !cycleDone) { 
            allCycles += timeBetweenStatesTimer.get();
            cycleNum ++;
            timeBetweenState = timeBetweenStatesTimer.get();
            timeBetweenStatesTimer.stop(); // TODO why you need the stop func?
            timeBetweenStatesTimer.reset();

            if (timeBetweenState > maxCycleTime) {
                maxCycleTime = timeBetweenState;
            }
            if (timeBetweenState < minCycleTime) { // TODO tha code can be change to two lines with max and min func
                minCycleTime = timeBetweenState;
            }
        }
        
        return timeBetweenState;
    }
    

    private static velocity getMaxVelocityTime() {
        if (Math.abs(currentVelocity.get()) > maxVelocity) {
            maxVelocity = currentVelocity.get();
        }
        if (Math.abs(Math.abs(currentVelocity.get()) - maxVelocity) < VELOCITY_TOLERANC) {
            timeInMaxSpeed.start();
            maxVelocityTime = timeInMaxSpeed.get(); 
        } else {
            timeInMaxSpeed.reset();
            timeInMaxSpeed.stop(); // TODO why you need this? 
        }

        return new velocity(maxVelocity, maxVelocityTime); // this need to return a defult value and the ccurrent code need to put in the if  and why use new?

    }

    private static void runCurrent() {
        for (int i = 0; i < subsystems.length; i++) {
            systemCurrents[i] += subsystems[i].getCurrent(); //TODO if you use this only as a temp varubal just call the subsystems[i].getCurrent() 
            //TODO the getcurrent isnt the suppley one its the stator one, you whant the supply
            currentPrecents[i] = new MeasureEntry(subsystems[i].getName(), systemCurrents[i] / sumOfCurrents * 100); // TODO instade of all the time create a new enrtry you can just add the systemcurrent/PDH.getTotalCurrent() * loop time right this code mean nothing in the pysical snese          //TODO why not use the getTotalCurrent func

            MALog.log("Performance Analyser/currents/" + subsystems[1].getName(), systemCurrents[i] / sumOfCurrents * 100); // TODO why not log the MeasureEntry
        }
    }

    private static double getTotalCurrent() {
        sumOfCurrents += PDH.getTotalCurrent(); //TODO this is also not coorect why its need to just return the getTotalCurrent
        return sumOfCurrents;
    }

    //TODO same problme as the last func
    private static void runVoltage() {
        for (int i = 0; i < subsystems.length; i++) {
            systemVoltage[i] += subsystems[i].getAppliedVolts();
            voltagePrecents[i] = new MeasureEntry(subsystems[i].getName(), systemVoltage[i] / sumOfVoltage * 100);

            MALog.log("Performance Analyser/currents/" + subsystems[1].getName(), systemCurrents[i] / sumOfVoltage * 100);
        }
    }

    private static double getTotalVoltage() {
        sumOfVoltage += PDH.getVoltage();
        return sumOfVoltage;
    }
    


    private static final acceleration getMaxAccelerationTime() {
        //TODO smae commandt as the velocity one
        accelerationTimer.start(); 
        currentAcceleration = (currentVelocity.get() - lastVelocity)/ (accelerationTimer.get() - lastTime);// TODO the pigion dont give you the accle?

        if(Math.abs(Math.abs(maxAcceleration) -  Math.abs(currentAcceleration)) < ACCLERATION_TOLERANC) {
            maxAcceleration = currentAcceleration;
            timerTimeInAcceleration.start();
            timeInAcceleration = timerTimeInAcceleration.get();
        }  else {
            timerTimeInAcceleration.reset();
            timerTimeInAcceleration.stop();
            timeInAcceleration = timerTimeInAcceleration.get();
        }

        lastTime = accelerationTimer.get();
        lastVelocity = currentVelocity.get();

        return new acceleration(maxAcceleration, timeInAcceleration);
    }

    //TODO why use such a complicated way, why dont use an init fucn in a builder stly and call the update in the robot periodiec, if a varubal got init yoy whant to analiz it
    public static void analaysCycleTimes(MRobotState enterFirstRobotState, MRobotState enterSecoundRobotState, Supplier<Boolean> startigCondition,
     Supplier<Boolean> enterEndCondition) {
        firstRobotState = enterFirstRobotState;
        secondRobotState = enterSecoundRobotState;

        firstCondition = startigCondition;
        endCondition = enterEndCondition;
        logCycleTimes = true;

    }

    public static void analaysTimeInCurrentState(Supplier<MRobotState> enterCurrentRobotState) {
        currentRobotState = enterCurrentRobotState;
        logTimeCurrentState = true;
    }

    public static void analaysAcceleration(Supplier<MRobotState> enterCurrentRobotState, Supplier<Double> enterCurrentVelocity) {
        currentRobotState = enterCurrentRobotState;
        currentVelocity = enterCurrentVelocity;
        logAcceleration =true;

    }

    public static void analaysMaxVelocity(Supplier<Double> enterCurrentVelocity) {
        currentVelocity = enterCurrentVelocity;
        logMaxVelocity = true;

    }

    public static void analaysCurrents(PowerControlledSystem... newSubsystems) {
        subsystems = newSubsystems;

        systemCurrents = new double[subsystems.length]; // TODO dont create all the time new array, its will fuck the code
        currentPrecents = new MeasureEntry[subsystems.length];

        logCurrent = true;

    }

    public static void analaysVoltage(PowerControlledSystem... newSubsystems) {
        subsystems = newSubsystems;

        if(systemVoltage == null) {
            systemVoltage = new double[subsystems.length];// TODO dont create all the time new array, its will fuck the code
        }

        if (voltagePrecents == null) {
            voltagePrecents = new MeasureEntry[subsystems.length];
        }

        logVoltage = true;

    }

    public static void update() { // call in robot priodic 
        if (logCycleTimes) {
            MALog.log("Performance Analyser/Current Cycle Time", getCycleTime());
            MALog.log("Performance Analyser/Max Cycle time", maxCycleTime);
            MALog.log("Performance Analyser/Min Cycle time",minCycleTime);

            MALog.log("Performance Analyser/Avrege Cycle Time", allCycles / cycleNum);

            MALog.log("Performance Analyser/mincycle", minCycleTime);



            MALog.log("Performance Analyser/Current State", currentRobotState.get().getStateName());
            MALog.log("Performance Analyser/is true", endCondition.get());


        }

        if (logTimeCurrentState) {
            MALog.log("Performance Analyser/Current Time In State", getTimeInCurrentState());
            MALog.log("Performance Analyser/Current State", currentRobotState.get().getStateName());

        }

        if (logMaxVelocity) {
            MALog.log("Performance Analyser/Max Velocity Time", getMaxVelocityTime().getVelocityTime());
            MALog.log("Performance Analyser/Max Velocity", getMaxVelocityTime().getVelocity());

        }

        if (logCurrent) {
            getTotalCurrent();
            runCurrent();
            System.out.println(PDH.getTotalCurrent());
        }

        if (logVoltage) {
            getTotalVoltage();
            runVoltage();
            System.out.println(PDH.getVoltage());
        }

        

        if (logAcceleration) {
            MALog.log("Performance Analyser/Max Acceleration time", getMaxAccelerationTime().getAccelerationTime());
            MALog.log("Performance Analyser/Max Acceleration", getMaxAccelerationTime().getAcceleration());
        }

    }


}
