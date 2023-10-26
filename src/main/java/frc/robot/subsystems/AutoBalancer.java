package frc.robot.subsystems;

import frc.robot.utils.Vector3;
import frc.robot.utils.PIDController;
import frc.robot.RobotConfig;


public class AutoBalancer {
    public static PIDController anglePID;
    public static double angleP;
    public static double angleI;
    public static double angleD;
    public static double angleDead;
    public static double angleVelDead;
    
    public static void init(){
        
        angleP = RobotConfig.angleP;
        angleI = RobotConfig.angleI;
        angleD = RobotConfig.angleD;
        angleDead = RobotConfig.angleDead;
        angleVelDead = RobotConfig.angleVelDead;

        anglePID = new PIDController( angleP, angleI, angleD, angleDead, angleVelDead, 1);
        anglePID.setDest(Math.PI / 2);

    }

    // public static void update() {
    //     double output = anglePID.updateOutput(getAngle());
    //     SwerveManager.rotateAndDrive(0, new Vector2(0, output));
    // }

    public static double updatePID() {
        Vector3 kHat = Pigeon.getKHat();
        double angle = kHat.getPitch();
        return anglePID.updateOutput(kHat.x < 0 ? angle : Math.PI - angle);
    } 

    public static boolean balanced() {
        return anglePID.atSetpoint();
    }
}