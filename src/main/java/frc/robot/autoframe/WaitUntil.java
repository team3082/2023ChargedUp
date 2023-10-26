package frc.robot.autoframe;


public class WaitUntil extends AutoFrame {
    private WaitFunc func;
    
    /**
    * Blocking. An atrocious frame Parker made that shouldn't ever be used. But in the off-chance it is, it blocks until 
    * a given lambda expression evaluates to true. 
    * @param func a function that will be evaluated every timestep, ending the WaitUntil frame if the result is true.
    */
    public WaitUntil(WaitFunc func) {
        this.blocking = true;
        this.func = func;
    }

    @Override
    public void update() {
        this.done = func.check();
    }

    public static interface WaitFunc {
        boolean check();
    }

}
