package frc.robot.autoframe;

import frc.robot.utils.PIDController;

public class WaitToDist extends AutoFrame {
    private PIDController targetController;
    private double stopDist;

    /**
    * Blocking. Waits until the error of a specified PIDController is below a given value. Units are relative to
    * the units used in the PIDController.
    * @param targetController the PIDController to check against. 
    * @param stopDist the error ar which the frame should stop blocking, expressed in the same units as 
    * those used in the PIDController.
    */
    public WaitToDist(PIDController targetController, double stopDist) {
        this.blocking = true;
        this.targetController = targetController;
        this.stopDist = stopDist;
    }

    @Override
    public void update() {
        if (Math.abs(targetController.getError()) < stopDist)
            this.done = true;
    }
}
