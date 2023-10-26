package frc.robot.autoframe;

import static frc.robot.Auto.activeFrames;

public class WaitToCompletion extends AutoFrame {
    private Class<? extends AutoFrame> targetClass;

    /**
     * Blocking. Waits until there are no remaining instances of a given AutoFrame in the set of active frames.
     * @param targetClass the class to check for. For example, to wait until all instances of MoveTo have
     * completed, you can write WaitToCompletion(MoveTo.class).
     */
    public WaitToCompletion(Class<? extends AutoFrame> targetClass) {
        this.blocking = true;
        this.targetClass = targetClass;
    }

    @Override
    public void update() {
        this.done = true;
        for (AutoFrame frame: activeFrames) {
            if (frame.getClass() == targetClass)
                this.done = false;
        }
    }
}
