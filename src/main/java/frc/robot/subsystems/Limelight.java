package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import frc.robot.utils.PIDController;

/**
 * A NetworkTables wrapper for the Limelight.
 */
public class Limelight {

    /** Enum representing various LED states of the limelight. */
    public enum LED {
        OFF(1),
        ON(3),
        BLINK(2);

        public double index;

        private LED(int index){
            this.index = index;
        }

    }
    
    /** Do we have a valid target? */
    public NetworkTableEntry targetDetected;

    /** 
     * Horizontal offset from limelight crosshair to target
     * -29.8 to 29.8 degrees.
     */
    public NetworkTableEntry xOffset;

    /**
     * Vertical offset from limelight crosshair to target
     * -24.85 to 24.85 degrees.
     */
    public NetworkTableEntry yOffset;

    /** Rotation or the "skew". -90 to 0 degrees. */
    public NetworkTableEntry targetRot;

    /** Target area. Rarely used in our context. 0% - 100% of image. */
    public NetworkTableEntry targetArea;

    /** LED mode of the limelight. */
    public NetworkTableEntry ledMode;

    /** PID controller intended for the limelight */
    public PIDController controller;

    /** A NetworkTables wrapper for the Limelight. */
    public Limelight() {

        // Find our NetworkTable
        NetworkTableInstance NT = NetworkTableInstance.getDefault();
        NetworkTable table = NT.getTable("limelight");

        // Fetch values from Limelight and input them into the table.
        targetDetected = table.getEntry("tv");
        xOffset = table.getEntry("tx");
        yOffset = table.getEntry("ty");
        targetRot = table.getEntry("ts");
        targetArea = table.getEntry("ta");
        ledMode = table.getEntry("ledMode");

        //setLED(LED.OFF);

    }

    /**
     * Set the LED state of the Limelight.
     * @param mode The desired state of the LEDs.
     */
    public void setLED(LED mode) {
        ledMode.setDouble(mode.index);
    }

    /**
     * Determine if a target is valid.
     * @return The validity of the target.
     */
    public boolean isTargetDetected() {
        return (targetDetected.getDouble(0) == 1);
    }

    /** Fetch the horizontal X offset. */
    public double getXOffset() {
        return xOffset.getDouble(0);
    }

    /** Fetch the vertical Y offset. */
    public double getYOffset() {
        return yOffset.getDouble(0);
    }

    /** Fetch the target's rotation/skew. */
    public double getRot() {
        return targetRot.getDouble(0);
    }

    /** Fetch the target area. */
    public double getArea() {
        return targetArea.getDouble(0);
    }

    
}
