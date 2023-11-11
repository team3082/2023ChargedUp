package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import frc.robot.subsystems.Arm.ArmPosition;

import frc.robot.utils.Vector2;

public class ArmProfileTest {
    
    //This test is intentionally turned off
    public void testArmProfile(){
        ProfileGenerator ap = new ProfileGenerator(ArmPosition.STARTING, ArmPosition.CONE_MIDDLE, 0.0005, 0.00001);
        ap.setDaemon(true);
        ap.setName("Arm Profile Test");
        ap.start();
        try{
            ap.join();
        }catch(InterruptedException e){
            System.out.println("Failed");
        }
        // for(double[] angles : ap.jointProfile){
        //     // if(Double.isNaN(angles[0])) break;
        //     System.out.printf("(%.3f, %.3f), ", angles[0], angles[1]);
        // }
        for(Vector2 position : ap.wristProfile){
            System.out.print(position + ", ");
        }
        assertEquals(ap.wristProfile.size(), ap.jointProfile.size());
        System.out.println(ap.wristProfile.size());
        System.out.println(ap.jointProfile.size());
        
    }

}
