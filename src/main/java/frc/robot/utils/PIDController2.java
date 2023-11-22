package frc.robot.utils;

public class PIDController2 {
    /**Proportional Error Coefficient */
    public final double kP;
    /**Integral Error Coefficient */
    public final double kI;
    /**Derivative Error Coefficient */
    public final double kD;
    
    /**Desired Position */
    private double desPos;
    /**Desired Change In Position */
    private double desVel;

    /**integral acculator */
    private double accumulator;
    /**Max Value for Integral Accumulator */
    private final double maxAccumulator;
    /** Previous Position, used to find velocity*/
    private double prevPos;

    public PIDController2(double kp, double ki, double kd){
        kP=kp;
        kI=ki;
        kD=kd;
        maxAccumulator = Double.MAX_VALUE;
        prevPos = Double.NaN;
    }

    public PIDController2(double kp, double ki, double kd, double maxAccumulator){
        kP=kp;
        kI=ki;
        kD=kd;
        this.maxAccumulator = maxAccumulator;
        prevPos = Double.NaN;
    }

    public void resetIntegralAccumulator(){
        accumulator = 0.0;
    }

    /**Call this only once per frame. Returns the desired acceleration */
    public double update(double pos){
        double error = desPos - pos;

        double vel = prevPos != Double.NaN ? (pos - prevPos) / RTime.deltaTime() : 0.0;
        prevPos = pos;

        accumulator += RTime.deltaTime() * error;
        accumulator = RMath.clamp(accumulator, -maxAccumulator, maxAccumulator);

        double output = kP * error + kD * (vel - desVel) + kI * accumulator;
        return output;
    }

    /**updates desired position and velocities, and returns the desired acceleration.
     * Only call this function once per frame;
     */
    public double update(double pos, double desPos, double desVel){
        this.desPos = desPos;
        this.desVel = desVel;

        return update(pos);
    }

    public double getError(double pos){
        return pos - desPos;
    }


}
