package frc.robot.autoframe;

public class RotateToGrid extends Rotate {
    /**
     * Rotates the robot to exactly 3π/2 radians so that it faces the grid. 
     */
    public RotateToGrid() {
        super((3*Math.PI)/2);
    }
}
