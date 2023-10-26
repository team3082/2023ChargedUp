package frc.robot.autoframe;

import frc.robot.subsystems.Manipulator;
import frc.robot.utils.RTime;

public class Outtake extends AutoFrame {
    private double duration;
    private double stopTime;

    /**
     * Instructs the manipulator to outtake for a set duration in seconds. 
     * @param duration the number of seconds the manipulator should stay active.
     */
    public Outtake(double duration){
        this.duration = duration;
    }

    public Outtake(){
        this(1.0);
    }

    @Override
    public void start(){
        stopTime = RTime.now() + duration;
        Manipulator.outtake();
    }

    @Override
    public void update(){
        if (RTime.now() > this.stopTime){
            this.done = true;
        }
    }

    @Override
    public void finish() {
        Manipulator.neutral();
    }
}
