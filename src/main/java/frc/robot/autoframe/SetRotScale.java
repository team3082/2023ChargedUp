package frc.robot.autoframe;

import frc.robot.Auto;

public class SetRotScale extends AutoFrame{

    private double scale = 1.0;

    public SetRotScale(double scale) {
        this.scale = scale;
    }

    @Override
    public void start(){
        Auto.rotScale = scale;
        this.done = true;
    }
}
