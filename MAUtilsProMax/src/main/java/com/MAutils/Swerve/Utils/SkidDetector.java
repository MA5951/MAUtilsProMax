package com.MAutils.Swerve.Utils;

import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Utils.Constants;
import com.MAutils.Utils.VectorUtil;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SkidDetector {
    private static final double SKID_THRESHOLD = 0.07; // TODO constance
    private static final double MIN_SKID_SPEED = 0.06;

    private final SwerveDriveKinematics kinematics;
    private final Supplier<SwerveModuleState[]> statesSupplier;
    private ChassisSpeeds measurSpeeds;
    private SwerveModuleState[] swerveStatesRotationalPart;
    private SwerveModuleState[] swerveStatesTranslationPart = new SwerveModuleState[4];
    private double[] swerveStatesTranslationalPartMagnitudes = new double[4];
    private boolean[] isSkidding = new boolean[] { false, false, false, false };
    private double[] skidRatio = new double[] { 0, 0, 0, 0 };
    private int lowestIndex = 0;
    private int secoundLowestIndex = 0;

    private Translation2d swerveStateMeasuredAsVector;
    private Translation2d swerveStatesRotationalPartAsVector;
    private Translation2d swerveStatesTranslationalPartAsVector = new Translation2d();
    private double minimumTranslationalSpeed = 0;
    private int numOfSkiddingModules = 0;

    public SkidDetector(SwerveSystemConstants constants, Supplier<SwerveModuleState[]> statesSupplier) {
        this.kinematics = constants.kinematics;
        this.statesSupplier = statesSupplier;
    }

    public void calculateSkid() {
        minimumTranslationalSpeed = Double.MAX_VALUE;
        measurSpeeds = kinematics.toChassisSpeeds(statesSupplier.get());
        swerveStatesRotationalPart = kinematics
                .toSwerveModuleStates(new ChassisSpeeds(0, 0, measurSpeeds.omegaRadiansPerSecond));

        // Calculate translational magnitudes and find minimum in one loop
        for (int i = 0; i < 4; i++) { // TODO change to statesSupplier.lenght

            swerveStateMeasuredAsVector = VectorUtil.getVectorFromSwerveState(statesSupplier.get()[i]);
            swerveStatesRotationalPartAsVector = VectorUtil.getVectorFromSwerveState(swerveStatesRotationalPart[i]);
            swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector
                    .minus(swerveStatesRotationalPartAsVector);

            swerveStatesTranslationPart[i] = new SwerveModuleState(
                    swerveStatesTranslationalPartAsVector.getNorm(),
                    swerveStatesTranslationalPartAsVector.getAngle());
            swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();

            if (swerveStatesTranslationalPartMagnitudes[i] < minimumTranslationalSpeed) {
                minimumTranslationalSpeed = swerveStatesTranslationalPartMagnitudes[i];
                secoundLowestIndex = lowestIndex;
                lowestIndex = i;
            }

        }

        MALog.logSwerveModuleStates("/Pose Estimator/Skid/Translation states", swerveStatesTranslationPart);
        MALog.log("Pose Estimator/Skid/Minimum Translational Speed",
                minimumTranslationalSpeed);

        if (Math.abs(minimumTranslationalSpeed) > MIN_SKID_SPEED) {

            numOfSkiddingModules = 0;

            for (int i = 0; i < 4; i++) {
                skidRatio[i] = Math
                        .abs((Math.abs(swerveStatesTranslationalPartMagnitudes[i] / minimumTranslationalSpeed)) - 1);

                isSkidding[i] = skidRatio[i] > SKID_THRESHOLD;

                if (isSkidding[i]) {
                    numOfSkiddingModules++;
                }

                MALog.log("Pose Estimator/Skid/Is Skidding/" + i, isSkidding[i]);
                MALog.log("Pose Estimator/Skid/Skid Ratios/" + i, skidRatio[i]); // TODO you can just log after all the
                                                                                 // if else
            }
        } else {
            for (int i = 0; i < 4; i++) {
                isSkidding[i] = false;
                skidRatio[i] = 0; // TODO insed of if for if else, you can just write for and then the
                                  // if(Math.abs(minimumTranslationalSpeed) > MIN_SKID_SPEED) insed in a ? : way

                MALog.log("Pose Estimator/Skid/Is Skidding/" + i, isSkidding[i]);
                MALog.log("Pose Estimator/Skid/Skid Ratios/" + i, skidRatio[i]);
            }
        }
    }

    public boolean[] getIsSkidding() {
        return isSkidding;
    }

    public int getNumOfSkiddingModules() {
        return numOfSkiddingModules;
    }

    public int getLowestIndex() {
        return lowestIndex;
    }

    public int getSecoundLowestIndex() {
        return secoundLowestIndex;
    }

}