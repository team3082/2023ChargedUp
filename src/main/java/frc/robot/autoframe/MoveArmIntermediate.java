package frc.robot.autoframe;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class MoveArmIntermediate extends MoveArm {
    
    private ArmPosition interPosition;

    public MoveArmIntermediate(ArmPosition interPos, ArmPosition finalPos) {
        super(finalPos);
        this.interPosition = interPos;
    }

    @Override
    public void start() {
        Arm.setStateIntermediate(interPosition, this.pos);
    }

}
