package frc.robot.autoframe;

/**
 * An abstract class that all other AutoFrames should inherit from.
 * 
 * The boolean "done" should be set to true when, as the name suggests, the frame is done executing. This is typically
 * set in the update() method.
 */
public abstract class AutoFrame {
    /**
     * Determines whether, when this frame is active, Auto will wait until its completion to activate any more frames. 
     * By default, frames are not blocking, meaning that Auto instantly queues subsequent frames after activating the 
     * frame in question. 
     */
    public boolean blocking = false;
    /**
     * Read by Auto each frame to determine whether the frame is done executing. By default, it is set to true the first
     * time update() runs, but this behavior can be overridden. 
     */
    public boolean done = false;

    /**
     * Runs immediately when a frame is activated. This should be overridden if the frame needs to set variables or execute
     * other one-time actions.  
     */
    public void start() {}

    /**
     * Runs every timestep between a frame's activation and a frame's completion. Most of the time, this function is where
     * you'll eventually want to set the boolean "done" to true, indicating that the frame is finished and can be removed
     * from the set of active frames. By default, "done" is set to true the first time it runs, but this behavior can be
     * overridden. 
     */
    public void update() {
        done = true;
    }

    /**
     * Runs one time when the frame is removed from the list of active frames (after "done" has been set to true). Most 
     * of the time, it's unnecessary to override this function. However, when a command must be sent to stop an action, 
     * it's preferable to send that command in finish() rather than in update(). This allows for frames like ClearActive,
     * which clears all of the active frames, to end these frames as if they had run to completion. 
     */
    public void finish() {}
}
