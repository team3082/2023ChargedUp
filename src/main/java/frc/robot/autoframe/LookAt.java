package frc.robot.autoframe;

import frc.robot.utils.Vector2;
import frc.robot.subsystems.swerve.SwervePosition;

public class LookAt extends Rotate {
    /**
    * Makes the robot look toward a specified (x, y) coordinate on the field.
    */
    public LookAt(double x, double y) {
        super(new Vector2(x, y).sub(SwervePosition.getPosition()).norm().atan2());
    }
}
