package frc.robot.autoframe;

import frc.robot.utils.Vector2;
import frc.robot.subsystems.swerve.SwervePID;
import static frc.robot.Auto.movement;

public class MoveTo extends AutoFrame {
    private PosComputeFunc func;
    private Vector2 move;
    /**
    * Moves the robot to a specified (x, y) coordinate on the field
    */
    public MoveTo(double x, double y) {
        this.move = new Vector2(x, y);
    }

    public MoveTo(Vector2 pos) {
        this.move = pos;
    }

    public MoveTo(PosComputeFunc func) {
        this.func = func;
        this.move = null;
    }

    @Override
    public void start() {
        if(this.move!=null)
            SwervePID.setDestPt(this.move);
        else
            SwervePID.setDestPt(this.func.run());
    }

    @Override
    public void update() {
        movement = SwervePID.updateOutputVel();
        if (SwervePID.atDest())
            this.done = true;
    }

    public static interface PosComputeFunc {
        Vector2 run();
    }
}
