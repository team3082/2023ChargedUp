package frc.robot.autoframe;

import frc.robot.subsystems.AutoBalancer;
import frc.robot.utils.Vector2;
import static frc.robot.Auto.movement;

public class Balance extends AutoFrame {
    /**
    Blocking. Enables PID to dock the robot on the charging station. Continues executing until the end of auto. 
    */
    public Balance() {
        this.blocking = true;
    }

    @Override
    public void update() {
        double output = AutoBalancer.updatePID();
        if (AutoBalancer.balanced())
            output = 0.0;
        movement = new Vector2(0, output);
    }
}
