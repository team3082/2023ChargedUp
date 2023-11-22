package frc.robot.subsystems.arm;

import frc.robot.utils.Vector2;

public enum ArmPosition {
    STARTING(new Vector2(4.75,8.5), Math.toRadians(45)),
    GROUND(new Vector2(27, 11), Math.toRadians(10)),
    PRIMED(new Vector2(15, 26), Math.toRadians(60)),
    SUBSTATION(new Vector2(28, 50.5), Math.toRadians(-2.5)),
    SUBSTATION_GRABBED(new Vector2(28, 50.5), Math.toRadians(10)),
    SUBSTATION_WAS_GRABBED(new Vector2(28, 50.5), Math.toRadians(-2.5)),

    //Cone Positions
    CONE_TOP(new Vector2(46, 60), Math.toRadians(60)),
    CONE_MIDDLE(new Vector2(30.75, 45), Math.toRadians(60)),
    CONE_BOTTOM(new Vector2(12, 10), 0),

    //Cube Positions
    CUBE_TOP(new Vector2(39, 50), 0.0),
    CUBE_MIDDLE(new Vector2(22.5, 35), 0.0),
    CUBE_BOTTOM(new Vector2(7.5, 18), 0.0),

    //Intermediate Positions
    GROUND_INTERMEDIATE(new Vector2(30, 30), Math.toRadians(45)),
    SCORE_INTERMEDIATE(new Vector2(22, 50), Math.toRadians(75));

    public final double[] angles;
    public final Vector2 position;
    public final double manipAng;

    ArmPosition(Vector2 pos, double manipAng) {
        position = pos;
        this.manipAng = manipAng;
        angles = ArmDynamics.inverseKinematics(this);
    }

    // private ArmPosition(double shldrT, double elbowT, double manipAng){
    //     position = ;
    //     angles = new double[]{shldrT, elbowT, manipAng - (shldrT + elbowT)};
    // }

}
