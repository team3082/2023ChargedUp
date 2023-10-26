package frc.robot.autoframe;

import frc.robot.utils.RTime;

public class Wait extends AutoFrame {
    private double duration;
    private double stopTime;

    /**
     * Blocking. Waits until a specified duration has elapsed, then allows Auto to resume execution. 
     * @param duration the duration, in seconds, to wait.
     */
    public Wait(double duration) {
        this.blocking = true;
        this.duration = duration;
    }

    @Override
    public void start() {
        this.stopTime = RTime.now() + duration;
    }

    @Override
    public void update() {
        if (RTime.now() > this.stopTime)
            this.done = true;
    }
}

