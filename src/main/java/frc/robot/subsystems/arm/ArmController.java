package frc.robot.subsystems.arm;

import java.util.function.BiFunction;

import org.ejml.simple.*;

import frc.robot.utils.RTime;

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
     * @param jointVector
     * @return
     */
    protected static SimpleMatrix calculateMassMatrix(double[] jv){
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

    protected static SimpleMatrix calculateMassMatrix2(double[] jv){
        //individual mass matrixes in coordinate space
        SimpleMatrix link0M = SimpleMatrix.diag(m0,m0,i0);
        SimpleMatrix link1M = SimpleMatrix.diag(m1,m1,i1);
        SimpleMatrix link2M = SimpleMatrix.diag(m2,m2,i2);

        //finding link mass matrices in joint space
        SimpleMatrix j0 = calculateJacobianLink0(jv);
        SimpleMatrix j1 = calculateJacobianLink1(jv);
        SimpleMatrix j2 = calculateJacobianLink2(jv);
        SimpleMatrix link0MassMatrix = j0.transpose().mult(link0M).mult(j0);
        SimpleMatrix link1MassMatrix = j1.transpose().mult(link1M).mult(j1);
        SimpleMatrix link2MassMatrix = j2.transpose().mult(link2M).mult(j2);
        
        return link0MassMatrix.plus(link1MassMatrix).plus(link2MassMatrix);
    }

    protected static SimpleMatrix calculateJacobianEndEffector(double[] jv){
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


    protected static SimpleMatrix calculateJacobianLink0(double[] jv){
        return new SimpleMatrix(3,3,true,new double[]{
            -r0 * Math.sin(jv[0]), 0,0,
            r0 * Math.cos(jv[0]), 0,0,
            0,0,0
        });
    }

    protected static SimpleMatrix calculateJacobianLink1(double[] jv){
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

    protected static SimpleMatrix calculateJacobianLink2(double[] jv){
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

    
}