package frc.robot.autoframe;

import frc.robot.subsystems.swerve.SwervePosition;

public class SetVision extends AutoFrame {
    
    private boolean isEnabled;

    public SetVision(boolean state) {
        this.isEnabled = state;
    }

    @Override
    public void start(){
        if(this.isEnabled){
            SwervePosition.enableVision();
        } else {
            SwervePosition.disableVision();
        }
        this.done = true;
    }
}
