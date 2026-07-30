package frc.robot.Subsystems.Swerve;

import com.MAutils.Swerve.Controllers.AngleAdjustController;
import com.MAutils.Swerve.Controllers.FieldCentricDrive;

import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.SwerveSystemConstants.GearRatio;
import com.MAutils.Swerve.SwerveSystemConstants.WheelType;
import com.MAutils.Swerve.Utils.PIDController;
import com.MAutils.Swerve.Utils.ProfiledPIDController;
import com.MAutils.Swerve.Utils.SwerveState;
import com.MAutils.Utils.GainConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.PortMap;
import frc.robot.RobotContainer;


public class SwerveConstants {

        public static double relSetPoint;

        public static final GainConfig driveGainConfig = new GainConfig().withKV(0.765).withKS(0.23).withKP(0);
        public static final GainConfig turnGainConfig = new GainConfig().withKP(0).withKS(0.23);

        public static final SlewRateLimiter setPointLimiterAbs = new SlewRateLimiter(1);
        public static final SlewRateLimiter setPointLimiterRel = new SlewRateLimiter(1);

        // Swerve System Constants
        public static final SwerveSystemConstants SWERVE_CONSTANTS = new SwerveSystemConstants()
                        .withPyshicalParameters(0.551, 0.551, 65, WheelType.BLACK_TREAD, 3.05)
                        .withMotors(DCMotor.getKrakenX60Foc(1), DCMotor.getFalcon500(1),
                                        PortMap.SwervePorts.SWERVE_MODULE_IDS,
                                        PortMap.SwervePorts.PIGEON2)
                        .withMaxVelocityMaxAcceleration(4.9, 10)
                        .withOdometryUpdateRate(250)
                        .withDriveCurrentLimit(55, true)
                        .withTurningCurrentLimit(50, true).withDriveTuning(driveGainConfig)
                        .withTurningTuning(turnGainConfig)
                        .withGearRatio(GearRatio.L2);

        // PID Controllers
        public static final PIDController ABS_PID_CONTROLLER = new PIDController(0.06, 0, 0)//0.06//0.09
                        .withContinuesInput(-180, 180)
                        .withTolerance(5);

        public static final PIDController ABS_PID_MOTION_CONTROLLER = new PIDController(0.11, 0, 0)//0.06//0.09
                        .withContinuesInput(-180, 180)
                        .withTolerance(7);

        public static final PIDController REL_PID_CONTROLLER = new PIDController(0.046, 0, 0)
                        .withContinuesInput(-180, 180)
                        .withTolerance(2);

        public static final ProfiledPIDController PROFILED_REL_PID_CONTROLLER = new ProfiledPIDController(5, 0, 0,
                        new Constraints(1000, 3300))// a= 500
                        .withContinuesInput(-180, 180)
                        .withTolerance(1.5);

        public static final PathConstraints constraints = new PathConstraints(
                        4, 3,
                        Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Swerve Drive Controllers
        public static final FieldCentricDrive FIELD_CENTRIC_DRIVE = new FieldCentricDrive(
                        RobotContainer.getDriverController(), SWERVE_CONSTANTS,
                        () -> Swerve.getInstance().getGyroData());


        // Swerve States
        public static final SwerveState NONE = new SwerveState("NONE").withXY(0, 0).withOmega(0);

        public static final SwerveState FIELD_CENTRIC = new SwerveState("Field Centric")
                        .withOnStateEnter(() -> FIELD_CENTRIC_DRIVE.withSclers(0.85, 0.35))
                        .withSpeeds(FIELD_CENTRIC_DRIVE);

        

        public static final SwerveState FIELD_CENTRIC_40 = new SwerveState("Field Centric 40 Precent")
                        .withOnStateEnter(() -> FIELD_CENTRIC_DRIVE.withSclers(0.3, 0.20))
                        .withSpeeds(FIELD_CENTRIC_DRIVE);

        
}