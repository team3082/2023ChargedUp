package frc.robot.subsystems;

import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.BezierCurve;
import frc.robot.utils.Vector2;

public class CurveFollow {

    public static double robotMaxSpeed = 120; //inches/sec
    public static BezierCurve followingCurve;
    public static int i;
    public static Vector2[] points;
    private static double cP = 0.1;

    public static void init(){
        //followingCurve = new BezierCurve(a, b, c, d, speed);
        points = followingCurve.getEqualSpacedPts();
        i = 0;
    }

    //This gets called 50 / sec
    public static void followCurve() {
        if(i==points.length-2)
            return;
        Vector2 vel = points[i+1].sub(points[i].div(1./50.)); //Inces/sec
        Vector2 correction = points[i].sub(SwervePosition.getPosition());
        correction = correction.mul(cP);
        vel = vel.add(correction);
        vel = vel.div(robotMaxSpeed);

        SwerveManager.rotateAndDrive(0, vel);
        
        i++;
    }
}
