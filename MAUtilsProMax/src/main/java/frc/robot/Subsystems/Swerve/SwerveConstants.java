package frc.robot.Subsystems.Swerve;

import com.MAutils.Swerve.Controllers.AngleAdjustController;
import com.MAutils.Swerve.Controllers.FieldCentricDrive;
import com.MAutils.Swerve.Controllers.XYAdjustControllerPID;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.SwerveSystemConstants.GearRatio;
import com.MAutils.Swerve.SwerveSystemConstants.WheelType;
import com.MAutils.Swerve.Utils.PIDController;
import com.MAutils.Swerve.Utils.ProfiledPIDController;
import com.MAutils.Swerve.Utils.SwerveController;
import com.MAutils.Swerve.Utils.SwerveState;
import com.MAutils.Utils.GainConfig;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.PortMap;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Util.Field;

public class SwerveConstants {


        public static final GainConfig driveGainConfig = new GainConfig().withKV(0.7).withKS(0.27).withKP(1);
        public static final GainConfig turnGainConfig = new GainConfig().withKP(55).withKS(0.3);

        // Swerve System Constants
        public static final SwerveSystemConstants SWERVE_CONSTANTS = new SwerveSystemConstants()
                        .withPyshicalParameters(0.56165, 0.56165, 27, WheelType.WCP_TREAD, 3.05)
                        .withMotors(DCMotor.getKrakenX60Foc(1), DCMotor.getKrakenX44(1),
                                        PortMap.SwervePorts.SWERVE_MODULE_IDS,
                                        PortMap.SwervePorts.PIGEON2)
                        .withMaxVelocityMaxAcceleration(5.303, 10)
                        .withOdometryUpdateRate(250)
                        .withDriveCurrentLimit(200, true)
                        .withTurningCurrentLimit(200, true).withDriveTuning(driveGainConfig)
                        .withTurningTuning(turnGainConfig)
                        .withGearRatio(GearRatio.L2MK5)
                        .withOptimize(false);

        // PID Controllers
        public static final PIDController ABS_PID_CONTROLLER = new PIDController(0.11, 0, 0)
                        .withContinuesInput(-180, 180)
                        .withTolerance(2);

        public static final PIDController REL_PID_CONTROLLER = new PIDController(0.065, 0, 0)
                        .withContinuesInput(-180, 180)
                        .withTolerance(0.5);

        public static final PIDController ABS_MOTION_PID_CONTROLLER = new PIDController(0.03, 0, 0)
                        .withContinuesInput(-180, 180)
                        .withTolerance(5);

        public static final PIDController REL_MOTION_PID_CONTROLLER = new PIDController(7, 0, 0)
                        .withContinuesInput(-180, 180)
                        .withTolerance(3);

        public static final PIDController REL_X_PID_CONTROLLER = new PIDController(3, 0, 0).withTolerance(0.05);// front // 3

        public static final PIDController REL_Y_PID_CONTROLLER = new PIDController(3.3, 0, 0).withTolerance(0.05);

        public static final PIDController ABS_X_PID_CONTROLLER = new PIDController(0, 0, 0).withTolerance(0.05);// front // 3.5

        public static final PIDController ABS_Y_PID_CONTROLLER = new PIDController(2.5, 0, 0).withTolerance(0.05);


        // Swerve Drive Controllers
        public static final FieldCentricDrive FIELD_CENTRIC_DRIVE = new FieldCentricDrive(
                        RobotContainer.getDriverController(), SWERVE_CONSTANTS,
                        () -> Swerve.getInstance().getGyroData());

        public static final AngleAdjustController ANGLE_ADJUST_CONTROLLER = new AngleAdjustController(SWERVE_CONSTANTS,
                        REL_PID_CONTROLLER);

        public static final XYAdjustControllerPID XY_ADJUST_CONTROLLER = new XYAdjustControllerPID(SWERVE_CONSTANTS, REL_X_PID_CONTROLLER,
         REL_Y_PID_CONTROLLER, ()-> VisionConstants.LL.getCameraIO().getPoseEstimate(PoseEstimateType.MT2).pose);
        // Swerve States
        public static final SwerveState NONE = new SwerveState("NONE").withXY(0, 0).withOmega(0);

        public static final SwerveState FIELD_CENTRIC = new SwerveState("Field Centric")
                        .withOnStateEnter(() -> FIELD_CENTRIC_DRIVE.withSclers(0.85, 0.35))
                        .withSpeeds(FIELD_CENTRIC_DRIVE);

        public static final SwerveState FIELD_CENTRIC_40 = new SwerveState("Field Centric 40 Precent")
                        .withOnStateEnter(() -> FIELD_CENTRIC_DRIVE.withSclers(0.3, 0.20))
                        .withSpeeds(FIELD_CENTRIC_DRIVE);

        public static final SwerveState ABS_CENTERING = new SwerveState("ABS Centering")
                        .withOnStateEnter(() -> {
                                ANGLE_ADJUST_CONTROLLER.withPIDController(ABS_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(clossest90());
                                ANGLE_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getAbsYawSupplier());

                        }).withSpeeds(ANGLE_ADJUST_CONTROLLER);


        public static final SwerveState REL_CENTRING = new SwerveState("REL Centring")
                        .withOnStateEnter(() -> {
                                ANGLE_ADJUST_CONTROLLER.withPIDController(REL_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(0);
                                ANGLE_ADJUST_CONTROLLER
                                                .withGyroSupplier(() -> VisionConstants.LL.getCameraIO().getTag().txnc);
                        }).withSpeeds(ANGLE_ADJUST_CONTROLLER);

        public static final SwerveState ABS_UNLOCKED = new SwerveState(" Absolute Unlocked")
                        .withOnStateEnter(() -> {
                                ANGLE_ADJUST_CONTROLLER.withPIDController(ABS_MOTION_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(() -> getAbsAngleToTargetFuter());
                                ANGLE_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getAbsYawSupplier());
                                FIELD_CENTRIC_DRIVE.withSclers(0.12, 0.1);
                        })
                        .withOmega(ANGLE_ADJUST_CONTROLLER)
                        .withXY(FIELD_CENTRIC_DRIVE);


        public static final SwerveState REL_UNLOCKED = new SwerveState("REL Unlocked")
                        .withOnStateEnter(() -> {
                                ANGLE_ADJUST_CONTROLLER.withPIDController(REL_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(0);
                                ANGLE_ADJUST_CONTROLLER.withGyroSupplier(()->VisionConstants.LL.getCameraIO().getTag().txnc);
                                FIELD_CENTRIC_DRIVE.withSclers(0.2, 0.1);
                        })
                        .withOmega(ANGLE_ADJUST_CONTROLLER)
                        .withXY(FIELD_CENTRIC_DRIVE);


        public static final SwerveState XY_ADJUST_REL = new SwerveState("XY Adjust REL").withOnStateEnter(() -> {
                                XY_ADJUST_CONTROLLER.withFieldRelative(true);
                                XY_ADJUST_CONTROLLER.withXYControllers(REL_X_PID_CONTROLLER, REL_Y_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withPIDController(ABS_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(180);
                                ANGLE_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getAbsYawSupplier());
                                XY_ADJUST_CONTROLLER.withXYSetPoint(() -> new Pose2d(3,4,new Rotation2d()), false);
                                XY_ADJUST_CONTROLLER.withMeasurment(()-> VisionConstants.LL.getCameraIO().getPoseEstimate(PoseEstimateType.MT2).pose);
                                XY_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getAbsYawSupplier());
                        })
                        .withXY(XY_ADJUST_CONTROLLER)
                        .withOmega(ANGLE_ADJUST_CONTROLLER);

        public static final SwerveState XY_ADJUST_ABS = new SwerveState("XY Adjust ABS").withOnStateEnter(() -> {
                                XY_ADJUST_CONTROLLER.withFieldRelative(true);
                                XY_ADJUST_CONTROLLER.withXYControllers(ABS_X_PID_CONTROLLER, ABS_Y_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withPIDController(ABS_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(180);
                                ANGLE_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getAbsYawSupplier());
                                XY_ADJUST_CONTROLLER.withXYSetPoint(() -> new Pose2d(3,4,new Rotation2d()), false);
                                XY_ADJUST_CONTROLLER.withMeasurment(()-> PoseEstimator.getCurrentPose());
                                XY_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getAbsYawSupplier());
                        })
                        .withXY(XY_ADJUST_CONTROLLER)
                        .withOmega(ANGLE_ADJUST_CONTROLLER);

        public static double getAbsAngleToTargetFuter() {

                double xDis = Field.getHub().getX()
                                - poseAdjust(
                                                PoseEstimator.getPoseLookAhead(1.1,
                                                                Swerve.getInstance().getChassisSpeeds()),
                                                VisionConstants.LL_OFFSET).getX();
                double yDis = Field.getHub().getY()
                                - poseAdjust(
                                                PoseEstimator.getPoseLookAhead(1.1,
                                                                Swerve.getInstance().getChassisSpeeds()),
                                                VisionConstants.LL_OFFSET).getY();
                double angle = Math.atan2(yDis, xDis);

                return Math.toDegrees(angle);
        }

        public static Translation2d poseAdjust(
                        Pose2d robotPoseField,
                        Translation2d offsetRobot) {
                return robotPoseField.getTranslation()
                                .plus(offsetRobot.rotateBy(robotPoseField.getRotation()));
        }


        public static double clossest90() {
                int index = (int)((Swerve.getInstance().getAbsYawSupplier().get()%360)/90);
                double angle = index * 90 + 90*(Math.signum((Swerve.getInstance().getAbsYawSupplier().get())));
                if (Math.abs(angle - Swerve.getInstance().getAbsYawSupplier().get())< 10) return Swerve.getInstance().getAbsYawSupplier().get()+90 ;
                return angle;
        }

}