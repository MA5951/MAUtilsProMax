
package frc.robot.Subsystems.Vision;

public class Vision {
    private static Vision vision;
     
    private Vision() {

    }

    public static Vision getInstance() {
        if(vision == null) {
            vision = new Vision();
        }
        return vision;
    }
} 
