package frc.robot.autoframe;

import frc.robot.subsystems.Manipulator;
import frc.robot.utils.RTime;


public class Intake extends AutoFrame {
    private double duration;
    private double stopTime;

    /**
     * Instructs the manipulator to intake for a set duration in seconds. 
     * @param duration the number of seconds the manipulator should stay active.
     */
    public Intake(double duration){
        this.duration = duration;
    }

    public Intake(){
        this(1.0);
    }

    @Override
    public void start(){
        stopTime = RTime.now() + duration;
        Manipulator.intake();
    }

    @Override
    public void update(){
        Manipulator.intake();
        if (RTime.now() > this.stopTime){
            this.done = true;
        }
    }

    @Override
    public void finish() {
        Manipulator.neutral();
    }
}
