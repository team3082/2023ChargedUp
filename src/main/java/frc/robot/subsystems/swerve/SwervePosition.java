package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Vision;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class SwervePosition {

    // Smoothly correct field position based on vision output. VISION_CORRECTION_FACTOR should range from 0.0 to
    // 1.0, representing the speed at which we blend from the odometry output to the output of the vision. 
    static final double VISION_CORRECTION_FACTOR = 0.1;

    private static Vector2 position;
    private static Vector2 absVelocity;
    private static Vector2 lastAbsVelocity;

    private static boolean correctWithVision = true;

    public static void init() {
        absVelocity     = new Vector2();
        lastAbsVelocity = new Vector2();
        position        = new Vector2();
    }

    public static void enableVision(){
        correctWithVision = true;
    }

    public static void disableVision(){
        correctWithVision = false;
    }

    public static void update() {

        // Derive our velocity 
        Vector2 vel = SwerveManager.getRobotDriveVelocity();

        //Telemetry.log(Severity.INFO, "drive: " + vel);

        // Rotate our velocity to be local to the field
        vel = vel.rotate(Pigeon.getRotationRad() - Math.PI / 2);

        // Flip the x component of our velocity if we're on the red alliance
        // I still don't know why, but we don't need to do this in simulation mode
        if (DriverStation.getAlliance() == Alliance.Red)
            vel.x *= -1;
        
        lastAbsVelocity = absVelocity; 
        absVelocity = vel;

        // Integrate our velocity to find our position
        position = position.add(absVelocity.add(lastAbsVelocity).mul(0.5 * RTime.deltaTime()));
        
        if (correctWithVision) {
            try {
                Vector2 visionPos = Vision.getRobotPos();
                Vector2 posError = visionPos.sub(position);
                position = position.add(posError.mul(VISION_CORRECTION_FACTOR));
            } catch(Exception e) {}
        }
    }

    private static final int numSamples = 50;
    private static Vector2[] lastSecond = new Vector2[numSamples];
    private static int index = 0;

    public static final double correctionMultiplier = 0.1;

    public static void updateAveragePosVision(){
        try {
            Vector2 visionPos = Vision.getRobotPos();
            Vector2 adjustment = visionPos.sub(position).mul(correctionMultiplier); //
            position = position.add(adjustment); //
            // lastSecond[index] = visionPos;
            // index = (index+1)%numSamples;
            // int i = 0;
            // Vector2 sum = new Vector2();
            // for(Vector2 v : lastSecond){
            //     if(v==null)
            //         break;
            //     sum = sum.add(v);
            //     i++;
            // }
            // position = sum.div(i);
        } catch(Exception e) {}
    }

    public static void updateAverageRotVision() {
        try {
            double visionRot = Vision.getRobotYaw();
            double adjustment = (visionRot - Pigeon.getRotationRad()) * correctionMultiplier;
            Pigeon.setYawRad(Pigeon.getRotationRad() + adjustment);
        } catch(Exception e) {}
    }

    //returns array of the robot's angle and distance in INCHES based of manual calculations
    public static double[] getPositionPolar(){
        
        Vector2 pos = getPosition();
        double distance = pos.mag();
        double angleRad = pos.atan2();

        return new double[]{angleRad, distance};
    }

    public static Vector2 getNetDisplacement(){
        return position;
    }

    public static Vector2 getPosition(){
        return position;
    }

    public static Vector2 getAbsVelocity() {
        return absVelocity;
    }

    /**
     * Recalibrates the SwervePosition based on a position on the field. Should only be used when vision is disabled,
     * otherwise it'll just be overwritten the next frame.
     * @param newPosition the new position to set the robot position to
     */
    public static void setPosition(Vector2 newPosition){
        position = newPosition;
    }

    //returns array of the robot's angle and distance in INCHES based of of sensor velocity
    // public static double[] angleDistancePositionVelocity(){
        
    //     double[] pos = new double[]{xPosition, yPosition};
        
    //     double distance = Math.sqrt(Math.pow(pos[0], 2) + Math.pow(pos[1], 2));
    //     double angle = Math.atan(pos[1]/pos[0])*(180/Math.PI);

    //     return new double[]{angle, distance};
        
    // }

}