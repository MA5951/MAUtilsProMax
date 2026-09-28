package frc.robot.Subsystems.MAcam;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PWM;

public class MAcam {

    // roboRIO DIO port
    private final DigitalInput input;
    private final DutyCycle pwm;

    // ESP32 sends 0-100% for 0-4000 mm
    private static final double MAX_DISTANCE_MM = 4000.0;

    public MAcam(int dioPort) {
        input = new DigitalInput(dioPort);
        pwm = new DutyCycle(input);

        
        
    }

    /**
     * Returns the distance measured by the VL53L1X in millimeters.
     */
    public double getDistanceMM() {
        double dutyCycle = pwm.getOutput();

        // Convert 0.0-1.0 duty cycle to 0-4000 mm
        return dutyCycle * MAX_DISTANCE_MM;
    }

    /**
     * Returns the PWM duty cycle.
     */
    public double getDutyCycle() {
        return pwm.getOutput();
    }

    /**
     * Free the roboRIO DIO resource.
     */
    public void close() {
        input.close();
    }
}
