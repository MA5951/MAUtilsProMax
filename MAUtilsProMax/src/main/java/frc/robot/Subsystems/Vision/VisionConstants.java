
package frc.robot.Subsystems.Vision;

import com.MAutils.Vision.Filters.FiltersConfig;
import com.MAutils.Vision.IOs.AprilTagCamera;
import com.MAutils.Vision.IOs.LimelightIO;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.Swerve.Swerve;


public class VisionConstants {
        public static final Translation2d LL_OFFSET = new Translation2d(0, 0.186);
        //public static final double FRONT_LL_HIGHT = 0.408;


        public static final FiltersConfig DEFAULT_FILTERS_CONFIG = new FiltersConfig();
               
        public static final AprilTagCamera LL = new AprilTagCamera(
                        new LimelightIO("limelight-frontll", Swerve.getInstance().getAbsYawSupplier()), DEFAULT_FILTERS_CONFIG,
                        Swerve.getInstance().getAbsYawSupplier(), () -> Swerve.getInstance().getChassisSpeeds(), () -> Swerve.getInstance().getGyroData().yawVelocity , 0.8);

}
