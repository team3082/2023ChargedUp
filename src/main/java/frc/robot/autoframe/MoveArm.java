package frc.robot.autoframe;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;

public class MoveArm extends AutoFrame {
    protected ArmPosition pos;
    
    /**
     * Moves the arm to a specified ArmPosition.
     * @param pos the ArmPosition to move to.
     */
    public MoveArm(ArmPosition pos){
        this.pos = pos;
    }

    @Override
    public void start() {
        Arm.setState(pos);
    }

    @Override
    public void update(){
        if(Arm.atPosition())
            this.done = true;
    }
}
