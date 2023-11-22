package frc.robot.subsystems.arm;

import frc.robot.utils.Vector2;

public enum ArmPosition {
    STARTING(new double[]{4.75,8.5, Math.toRadians(45)}),
    GROUND(new double[]{27, 11, Math.toRadians(10)}),
    PRIMED(new double[]{15, 26, Math.toRadians(60)}),
    SUBSTATION(new double[]{28, 50.5, Math.toRadians(-2.5)}),
    SUBSTATION_GRABBED(new double[]{28, 50.5, Math.toRadians(10)}),
    SUBSTATION_WAS_GRABBED(new double[]{28, 50.5, Math.toRadians(-2.5)}),

    //Cone Positions
    CONE_TOP(new double[]{46, 60, Math.toRadians(60)}),
    CONE_MIDDLE(new double[]{30.75, 45, Math.toRadians(60)}),
    CONE_BOTTOM(new double[]{12, 10, 0}),

    //Cube Positions
    CUBE_TOP(new double[]{39, 50, 0.0}),
    CUBE_MIDDLE(new double[]{22.5, 35, 0.0}),
    CUBE_BOTTOM(new double[]{7.5, 18, 0.0}),

    //Intermediate Positions
    GROUND_INTERMEDIATE(new double[]{30, 30, Math.toRadians(45)}),
    SCORE_INTERMEDIATE(new double[]{22, 50, Math.toRadians(75)});

    /**joint vector (q0,q1,q2) */
    public final double[] jv;
    /**cartesian vector wrt the shoulder (x,y,theta) */
    public final double[] cv;

    /**
     * @param cv cartesian vector wrt the base of the robot
     */
    ArmPosition(double[] cv) {
        if(cv.length != 3) throw new IllegalArgumentException("Something isn't quite right here");
        this.cv = ArmDynamics.robotToShoulder(cv);
        this.jv = ArmDynamics.inverseKinematics(this.cv);
    }

}
