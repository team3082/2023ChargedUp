package frc.robot.autoframe;

import frc.robot.Auto;
import frc.robot.RobotConfig;

public class SetMoveScale extends AutoFrame{

    private double scale = 1.0;

    public SetMoveScale(double scale) {
        this.scale = scale;
    }

    /** 
     * Sets the absolute scale (ignores the clamping done by SwervePID)
     */
    public static SetMoveScale setAbsoluteMoveScale(double absScale) {
        return new SetMoveScale(absScale/RobotConfig.moveSpeedMax);
    }

    @Override
    public void start(){
        Auto.moveScale = scale;
        this.done = true;
    }
}
