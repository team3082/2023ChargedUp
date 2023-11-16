package frc.robot.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import frc.robot.subsystems.ProfileGenerator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.utils.Vector2;
import java.util.LinkedList;
import java.util.Timer;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import java.util.concurrent.TimeUnit;

public class ArmTest{

    private static final double IK_tolerance = 0.001; 

    public boolean IKtest(double x, double y, double shldrExpected, double elbowExpected){
        double[] a = Arm.calculateAngles(new Vector2(x,y),0);
        // Telemetry.log(Telemetry.Severity.INFO, a[0]);
        // Telemetry.log(Telemetry.Severity.INFO, a[1]);
        return  Math.abs(a[0]-shldrExpected)<IK_tolerance && 
                Math.abs(a[1]-elbowExpected)<IK_tolerance;
    }

    @Test
    public void validateBasicInverseKinematics(){
        //In front
        //Horizontal
        assertTrue(IKtest(55, 37, 0, 0));
        //Below
        assertTrue(IKtest(48.75, 52, -0.082, 0.786));
        assertTrue(IKtest(31.75, 40, -1.048, 2.061));
        assertTrue(IKtest(12,30.5, -2.839,2.841));
        // //Above
        // assertTrue(IKtest(35, 80, 0.197, 1.124));
        // //Behind
        // //Below horz.
        // assertTrue(IKtest(0, 15, -2.558, 1.439));
    }

    @Test
    public void testProfile(){
        BufferedTrajectoryPointStream[] streams = new BufferedTrajectoryPointStream[]{new BufferedTrajectoryPointStream(), new BufferedTrajectoryPointStream(), new BufferedTrajectoryPointStream()};
        ProfileGenerator pg = new ProfileGenerator(ArmPosition.CONE_MIDDLE.position, ArmPosition.STARTING.position, ArmPosition.STARTING.angles[2], 5, 50);
        // LinkedList<Vector2> wristProfile = pg.generateWristProfile(ArmPosition.CONE_MIDDLE.position, ArmPosition.STARTING.position, 25,100);
        // System.out.println(wristProfile);
        pg.start();
        // pg.start();
        // pg.generateJointProfile(ArmPosition.CONE_MIDDLE.position, ArmPosition.STARTING.angles[2], ArmPosition.STARTING.position, 5, 50);
        // try{
        //     // pg.join();
        //     System.out.println("threads linked");
        // }catch(Exception e){
        //     //DO nothing
        //     System.out.println("something is wrong");
        // }      
        try{
            pg.join();
        }catch(InterruptedException e){
            e.printStackTrace();
        }
        System.out.println("done");  
    }
}
    
