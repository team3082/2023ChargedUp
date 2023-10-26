package frc.robot.subsystems.swerve;

import frc.robot.utils.PIDController;
import frc.robot.utils.Vector3;
import frc.robot.utils.Vector2;
import frc.robot.subsystems.Pigeon;
import frc.robot.RobotConfig;

public class TipProtection {

    private static PIDController pid;
    private static double kP;
    private static double kI;
    private static double kD;
    private static double dead;
    private static double velDead;
    private static double max;
    public static boolean isEnabled;
    
    /**
     * initializes the Tip protection pid controller, and enables or disables Tip protection
     * @param enable whether or not to enabled the tip protection
     */
    public static void init(boolean enable){
        kP = RobotConfig.tipP;
        kI = RobotConfig.tipI;
        kD = RobotConfig.tipD;
        dead = RobotConfig.tipDead;
        velDead = RobotConfig.tipVelDead;
        max = RobotConfig.tipMax;

        pid = new PIDController(kP,kI,kD,dead,velDead,max);
        pid.setDest(Math.PI / 2);
        isEnabled = enable;
    }

    /**
     * Returns a vector to be added to the joystick input
     * <p>
     * Note: Calling this method may return a non-zero vector even when disabled
     * <p/>
     * @return The vector to add to the input
     */
    public static Vector2 updateOutput(){
        Vector3 kHat = Pigeon.getKHat();
        double pitch = kHat.getPitch();

        //This is the magnitude of the corrections
        double mag = pid.updateOutput(pitch);

        //unit vector in the tipping direction
        Vector2 dir = new Vector2(kHat.x, kHat.y);
        dir.norm();

        return dir.mul(mag).rotate(-Pigeon.getRotationRad());
        // return new Vector2();
    }

    /**
     * Whether or not the Tip protection is trying to correct
     * @return Whether or not the robot is tipping
     */
    public static boolean isTipping(){
        

        return Math.abs(pid.getError()) > dead;
        
    }


}
