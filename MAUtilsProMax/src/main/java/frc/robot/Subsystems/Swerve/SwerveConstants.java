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


        public static final GainConfig driveGainConfig = new GainConfig().withKV(0).withKS(0).withKP(0.3);
        public static final GainConfig turnGainConfig = new GainConfig().withKP(10).withKS(0);

        public static final SlewRateLimiter setPointLimiterAbs = new SlewRateLimiter(1);
        public static final SlewRateLimiter setPointLimiterRel = new SlewRateLimiter(1);

        // Swerve System Constants
        public static final SwerveSystemConstants SWERVE_CONSTANTS = new SwerveSystemConstants()
                        .withPyshicalParameters(0.56165, 0.56165, 65, WheelType.WCP_TREAD, 3.05)
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