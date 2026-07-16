
package com.MAutils.PoseEstimation;

import java.util.NoSuchElementException;
import java.util.Optional;

import com.MAutils.Logger.MALog;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;

public class PoseEstimationMA {
  public static PoseEstimationMA instance;

  private static final double poseBufferSizeSec = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

  private Pose2d odometryPose = Pose2d.kZero;
  private Pose2d estimatedPose = Pose2d.kZero;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
  private final TimeInterpolatableBuffer<Rotation3d> rotationBuffer = TimeInterpolatableBuffer
      .createBuffer(poseBufferSizeSec);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  private Rotation2d gyroOffset = Rotation2d.kZero;

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();
  private ChassisSpeeds robotSetpointVelocity = new ChassisSpeeds();

  public static PoseEstimationMA getInstance() {
    if (instance == null) {
      instance = new PoseEstimationMA();
    }
    return instance;
  }

  public PoseEstimationMA() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
  }

  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated -
    // offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
  }

  public Pose2d getEstimatPose() {
    return estimatedPose;
  }

  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  public void addOdometryObservation(OdometryObservation observation, SwerveDriveKinematics kinematics) {
    // Logic to scale down odometry
    var t = 1.0;
    if (observation.pitch().isPresent() && observation.roll().isPresent()) {
      t = 1
          - (MathUtil.inverseInterpolate(
              0,
              25,
              Math.abs(
                  Units.radiansToDegrees(
                      Math.acos(
                          observation.pitch().get().getCos()
                              * observation.roll().get().getCos())))));
    }
    t = MathUtil.clamp(t, 0.0, 1.0);

    // Update odometry pose
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    twist = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
    lastWheelPositions = observation.wheelPositions().clone();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);

    // Replace odometry pose with gyro if present
    observation.yaw.ifPresent(
        gyroAngle -> {
          // Add offset to measured angle
          Rotation2d angle = gyroAngle.plus(gyroOffset);
          odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });

    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);

    // Add rotation to buffer at timestamp
    if (observation.roll.isPresent()) {
      rotationBuffer.addSample(
          observation.timestamp(),
          new Rotation3d(
              observation.roll.get().getRadians(),
              observation.pitch.get().getRadians(),
              observation.yaw.get().getRadians()));
    }
    // Apply odometry delta to vision pose estimate
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  /** Adds a new vision pose observation from the vision subsystem. */
  public void addVisionObservation(VisionObservation observation, String cameraName) {
    MALog.log("PoseEstimationMA/VisionObservation/" + cameraName + "/Pose", observation.visionPose());
    MALog.log("PoseEstimationMA/VisionObservation/" + cameraName + "/Timestamp", observation.timestamp());

    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
        System.out.println("Skipping vision measurement because it is too old to be in the buffer" + cameraName);
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // Calculate transforms between odometry pose and vision sample pose
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

    // Shift estimated pose backwards to sample time
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    // Calculate the transform from the shifted estimate to the observation pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

    // Scale the transform by the Kalman gain
    var kTimesTransform = visionK.times(
        VecBuilder.fill(
            transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform = new Transform2d(
        kTimesTransform.get(0, 0),
        kTimesTransform.get(1, 0),
        Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate the current estimate by applying the scaled transform to the old
    // estimate
    // then shifting forwards using odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  

  // MARK: - Type declarations

  public record OdometryObservation(
      double timestamp,
      SwerveModulePosition[] wheelPositions,
      Optional<Rotation2d> roll,
      Optional<Rotation2d> pitch,
      Optional<Rotation2d> yaw) {
  }

  public record VisionObservation(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {
  }

  public void update() {
    MALog.log("PoseEstimationMA/EstimatedPose", estimatedPose);
    MALog.log("PoseEstimationMA/OdometryPose", odometryPose);
  }
}
