package frc.robot.autoframe;

import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.Vector2;

public class SetOdometryPos extends AutoFrame {    
    private Vector2 newPos;
    
    public SetOdometryPos(Vector2 pos){
        this.newPos = pos;
    }

    @Override
    public void start(){
        if(newPos!=null)
            SwervePosition.setPosition(newPos);
        this.done = true;
    }
}