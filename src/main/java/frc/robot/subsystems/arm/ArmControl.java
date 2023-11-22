package frc.robot.subsystems.arm;


import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.PIDController2;
import frc.robot.utils.Vector2;

public class ArmControl {
    static final double j0kP = 0.0;
    static final double j0kI = 0.0;
    static final double j0kD = 0.0;

    static final double j1kP = 0.0;
    static final double j1kI = 0.0;
    static final double j1kD = 0.0;

    static final double j2kP = 0.0;
    static final double j2kI = 0.0;
    static final double j2kD = 0.0;

    static PIDController2 j0Controller = new PIDController2(j0kP, j0kI, j0kD);
    static PIDController2 j1Controller = new PIDController2(j1kP, j1kI, j1kD);
    static PIDController2 j2Controller = new PIDController2(j2kP, j2kI, j2kD);

    private static ControlMode currentMode = ControlMode.POSITION;

    private static double[] desJv;

    public enum ControlMode {
        POSITION, NEUTRAL;
    }

    /**only call once per frame.  */
    public static double[] update(double[] jv){

        switch(currentMode){
            case POSITION:
                return updatePositionMode(jv);
            case NEUTRAL:
                return updateNeutralMode(jv);
            default:
                throw new RuntimeException("Something Wrong");
        }
    }

    /**Returns the desired torque values to make the arm approach the set point */
    private static double[] updatePositionMode(double[] jv){
        double a0 = j0Controller.update(jv[0]);
        double a1 = j1Controller.update(jv[1]);
        double a2 = j2Controller.update(jv[2]);

        double[] desAccel = {a0,a1,a2};

        double[] desTorques = ArmDynamics.calcTorques(jv, desAccel);
        return desTorques;
    }

    private static double[] updateNeutralMode(double[] jv){
        return Arrays.copyOf(ArmDynamics.calcGravityTorques(jv).getDDRM().data, 3);
    }

    public static void setModePosition(double[] desPos){
        currentMode = ControlMode.POSITION;
        desJv = ArmDynamics.inverseKinematics(desPos);
        j0Controller.setDest(desJv[0]);
        j1Controller.setDest(desJv[1]);
        j2Controller.setDest(desJv[2]);
    }

    public static void setModePosition(ArmPosition desPos){
        setModePosition(desPos.cv);
    }

    public static void setModeNeutral(){
        currentMode = ControlMode.NEUTRAL;
    }
}
