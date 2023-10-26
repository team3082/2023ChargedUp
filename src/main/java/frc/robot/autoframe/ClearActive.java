package frc.robot.autoframe;

import static frc.robot.Auto.activeFrames;

public class ClearActive extends AutoFrame {
    /**
     * Clears all active frames and calls their finish() functions. 
     */
    @Override
    public void start() {
        for (AutoFrame frame : activeFrames) {
            frame.finish();
        }
        activeFrames.clear();
    }
}
