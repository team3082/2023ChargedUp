package frc.robot.autoframe;

import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.swerve.SwervePID;
import static frc.robot.Auto.rotSpeed;

public class Rotate extends AutoFrame {
    private RotComputeFunc func;
    private double targetAngle;

    /**
    * Makes the robot rotate to a specified angle
    * @param rad angle in radians
    */
    public Rotate(double rad) {
        this.targetAngle = rad;
        this.func = null;
    }

    public Rotate(RotComputeFunc func) {
        this.func = func;
    }

    @Override
    public void start() {
        if(this.func==null)
            SwervePID.setDestRot(this.targetAngle);
        else
            SwervePID.setDestRot(func.run());
    }

    @Override
    public void update() {
        rotSpeed = SwervePID.updateOutputRot()*.5;
        Telemetry.log(Telemetry.Severity.INFO,"error: " + SwervePID.rotPID.getError());
        if (SwervePID.atRot())
            this.done = true;
    }

    public static interface RotComputeFunc {
        double run();
    }
}
