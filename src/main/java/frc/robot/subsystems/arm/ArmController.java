package frc.robot.subsystems.arm;

import java.util.function.BiFunction;

import org.ejml.simple.*;

import frc.robot.utils.RTime;

//all units are terms of inches, seconds, pounds, and radians
public class ArmController {
    //Constants. All units in inches, secounds, and pounds
    //TODO get moment of inertia values
    static double g = -386.0885826772;

    static double l0 = 22.0;//joint length
    static double r0 = 15.0;//COM distance from joint
    static double i0 = 0.0;//moment of inertia about com
    static double m0 = 6;//mass

    static double l1 = 26.0;
    static double r1 = 18.0;
    static double i1 = 0.0;
    static double m1 = 2.68;

    static double l2 = 12.0;
    static double r2 = 6.0;
    static double i2 = 0.0;
    static double m2 = 8.3125;
    
    /**
     * calculates the inertia matrix, which transforms joint accelerations to joint torques
     * @param jointVector vector containing the joint angles
     * @return joint space inertia matrix
     */
    protected static SimpleMatrix calcMassMatrix(double[] jv){
        //This is going to be stinky, so don't bother debugging
        double c1 = Math.cos(jv[1]);
        double c2 = Math.cos(jv[2]);
        double c12 = Math.cos(jv[1]+jv[2]);

        //making link 2 matrix
        double a11 = m2*(Math.pow(r2, 2) + Math.pow(l1,2) + Math.pow(l0,2)+ 2 * l1 * r2 * c2 + 2 * l1 * l0 * c1 + 2 * l0 * r2 * c12) + i2;
        double a22 = m2*(Math.pow(r2, 2) + Math.pow(l1,2) + 2 * l1 * r2 * c2) + i2;
        double a12 = m2*(Math.pow(r2, 2) + Math.pow(l1,2) + 2 * l1 * r2 * c2 + l0 * r2 * c12 + l0 * l1 * c1) + i2;
        double a13 = m2*(Math.pow(r2, 2) + l1 * r2 * c2 + l0 * r2 * c12) + i2;
        double a33 = m2*Math.pow(r2,2) + i2;
        double a23 = m2*(Math.pow(r2, 2) + l1 * r2 * c2) + i2;

        SimpleMatrix link2MassMatrix = new SimpleMatrix(new double[][]{
            {a11,a12,a13},
            {a12,a22,a23},
            {a13,a23, a33}
        });

        //link 1 matrix
        a11 = m1*(Math.pow(r1,2) + Math.pow(l0,2) + 2 * r1 * l0 * c1) + i1;
        a12 = m1*(Math.pow(r1,2) + r1 * l0 * c1) + i1;
        a22 = m1*Math.pow(r1,2) + i1;
        
        SimpleMatrix link1MassMatrix = new SimpleMatrix(new double[][]{
            {a11,a12,0},
            {a12,a22,0},
            {0,0,0}
        });

        //link 0 matrix
        a11 = m0*Math.pow(r0,2) + i0;
        SimpleMatrix link0MassMatrix = new SimpleMatrix(3,3, true, new double[]{a11,0,0,0,0,0,0,0,0});

        return link0MassMatrix.plus(link1MassMatrix).plus(link2MassMatrix);
    }

    /**
     * calculates the inertia matrix, which transforms joint accelerations to joint torques
     * @param jointVector vector containing the joint angles
     * @return joint space inertia matrix
     */
    protected static SimpleMatrix calcMassMatrix2(double[] jv){
        //individual mass matrixes in coordinate space
        SimpleMatrix link0M = SimpleMatrix.diag(m0,m0,i0);
        SimpleMatrix link1M = SimpleMatrix.diag(m1,m1,i1);
        SimpleMatrix link2M = SimpleMatrix.diag(m2,m2,i2);

        //finding link mass matrices in joint space
        SimpleMatrix j0 = calcJacobianLink0(jv);
        SimpleMatrix j1 = calcJacobianLink1(jv);
        SimpleMatrix j2 = calcJacobianLink2(jv);
        SimpleMatrix link0MassMatrix = j0.transpose().mult(link0M).mult(j0);
        SimpleMatrix link1MassMatrix = j1.transpose().mult(link1M).mult(j1);
        SimpleMatrix link2MassMatrix = j2.transpose().mult(link2M).mult(j2);
        
        return link0MassMatrix.plus(link1MassMatrix).plus(link2MassMatrix);
    }

    /** 
     * Calculates the jacobian for the endeffector. Relates joint space velocities to operational space velocities
     * @param jv joint angle vector
     * @return endeffector javobian
    */
    protected static SimpleMatrix calcJacobianEndEffector(double[] jv){
        double s012 = Math.sin(jv[0] + jv[1] + jv[2]);
        double s01 = Math.sin(jv[0] + jv[1]);
        double c012 = Math.cos(jv[0] + jv[1] + jv[2]);
        double c01 = Math.cos(jv[0] + jv[1]);

        return new SimpleMatrix(3,3,true, new double[]{
            -l2 * s012 - l1 * s01 - l0 * Math.sin(jv[0]), -l2 * s012 - l1 * s01, -l2 * s012,
            l2 * c012 + l1 * c01 + l0 * Math.cos(jv[0]), l2 * c012 + l1 * c01, l2 * c012,
            1,1,1
        });
    }

    /**
     * calculates jacobian for link 0 COM. Relates joint velocities to cartesian velocities of the com
     * @param jv joint vector
     * @return Link zero COM jacobian
     */
    protected static SimpleMatrix calcJacobianLink0(double[] jv){
        return new SimpleMatrix(3,3,true,new double[]{
            -r0 * Math.sin(jv[0]), 0,0,
            r0 * Math.cos(jv[0]), 0,0,
            0,0,0
        });
    }

    /**
     * calculates jacobian for link 1 COM. Relates joint velocities to cartesian velocities of the com
     * @param jv joint vector
     * @return Link one COM jacobian
     */
    protected static SimpleMatrix calcJacobianLink1(double[] jv){
        double s01 = Math.sin(jv[0] + jv[1]);
        double s0 = Math.sin(jv[0]);
        double c01 = Math.cos(jv[0] + jv[1]);
        double c0 = Math.cos(jv[0]);

        return new SimpleMatrix(3,3,true, new double[]{
            -r1 * s01 - l0 * s0, - r1 * s01, 0,
            r1 * c01 + l0 * c0, r1 * c01, 0,
            1,1,0
        });
    }

    /**
     * calculates jacobian for link 2 COM. Relates joint velocities to cartesian velocities of the com
     * @param jv joint vector
     * @return Link two COM jacobian
     */
    protected static SimpleMatrix calcJacobianLink2(double[] jv){
        double s012 = Math.sin(jv[0] + jv[1] + jv[2]);
        double s01 = Math.sin(jv[0] + jv[1]);
        double c012 = Math.cos(jv[0] + jv[1] + jv[2]);
        double c01 = Math.cos(jv[0] + jv[1]);

        return new SimpleMatrix(3,3,true, new double[]{
            -r2 * s012 - l1 * s01 - l0 * Math.sin(jv[0]), -r2 * s012 - l1 * s01, -r2 * s012,
            r2 * c012 + l1 * c01 + l0 * Math.cos(jv[0]), r2 * c012 + l1 * c01, r2 * c012,
            1,1,1
        });
    }

    /**
     * calculates matrix to convert from elbow coordinates to base coordinates
     * @param jv joint vector
     * @return elbow to base matrix
     */
    protected static SimpleMatrix calcForwardMatrixb0(double[] jv){
        double c0 = Math.cos(jv[0]);
        double s0 = Math.sin(jv[0]);
        return new SimpleMatrix(4,4, true, new double[]{
            c0, -s0, 0, l0 * c0,
            s0, c0, 0, l0 * s0,
            0, 0, 1, jv[0],
            0,0,0,1
        });
    }

    /**
     * calculates matrix to convert from wrist coordinates to elbow coordinates
     * @param jv joint vector
     * @return wrist to elbow matrix
     */
    protected static SimpleMatrix calcForwardMatrix01(double[] jv){
        double c1 = Math.cos(jv[1]);
        double s1 = Math.sin(jv[1]);
        return new SimpleMatrix(4,4, true, new double[]{
            c1, -s1, 0, l1 * c1,
            s1, c1, 0, l1 * s1,
            0, 0, 1, jv[1],
            0,0,0,1
        });
    }

    /**
     * calculates matrix to convert from end effector coordinates to wrist coordinates
     * @param jv joint vector
     * @return End effector to wrist matrix
     */
    protected static SimpleMatrix calcForwardMatrix12(double[] jv){
        double c2 = Math.cos(jv[2]);
        double s2 = Math.sin(jv[2]);
        return new SimpleMatrix(4,4, true, new double[]{
            c2, -s2, 0, l2 * c2,
            s2, c2, 0, l2 * s2,
            0, 0, 1, jv[2],
            0,0,0,1
        });
    }

    /**
     * calculates matrix to convert from endeffector coordinates to base coordinates
     * @param jv joint vector
     * @return Forward Kinematics Matrix
     */
    protected static SimpleMatrix calcForwardMatrixb2(double[] jv){
        SimpleMatrix tb0 = calcForwardMatrixb0(jv);
        SimpleMatrix t01 = calcForwardMatrix01(jv);
        SimpleMatrix t12 = calcForwardMatrix12(jv);

        return tb0.mult(t01).mult(t12);
    }

    /**
     * returns a 3x1 column vector containing the torques created by gravity
     * @param jv joint vector
     * @return gravity torques
     */
    protected static SimpleMatrix calcGravityTorques(double[] jv){
        SimpleMatrix j0 = calcJacobianLink0(jv);
        SimpleMatrix j1 = calcJacobianLink1(jv);
        SimpleMatrix j2 = calcJacobianLink2(jv);


        
        SimpleMatrix fg0 = new SimpleMatrix(3,1,false,new double[]{0,-m0,0});
        SimpleMatrix fg1 = new SimpleMatrix(3,1,false,new double[]{0,-m1,0});
        SimpleMatrix fg2 = new SimpleMatrix(3,1,false,new double[]{0,-m2,0});

        SimpleMatrix fg = j0.transpose().mult(fg0).plus(j1.transpose().mult(fg1)).plus(j2.transpose().mult(fg2));
        return fg;
    }

    /**
     * calculates the needed motor torques to produce a given acceleration in joint space
     */
    protected static SimpleMatrix calcTorques(double[] jv, double[] desAccel){
        SimpleMatrix mass = calcMassMatrix(jv);
        SimpleMatrix gravity = calcGravityTorques(jv);
        SimpleMatrix accel = new SimpleMatrix(3,1,false,desAccel);

        SimpleMatrix motorTorques = mass.mult(accel).minus(gravity);
        return motorTorques;
    }

    
}
