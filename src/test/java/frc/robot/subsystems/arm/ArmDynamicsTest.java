package frc.robot.subsystems.arm;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.ejml.simple.*;
import org.junit.jupiter.api.Test;

public class ArmDynamicsTest {

    @Test
    public void massMatrixTest(){
        double[] jv = {2,1,2};
        try{
        var m1 = ArmDynamics.calcMassMatrix(jv);
        var m2 = ArmDynamics.calcMassMatrix2(jv);
        // System.out.println(m1);
        // System.out.println(m2);
        assertTrue(m1.isIdentical(m2, 1));
        }catch(RuntimeException e){
            e.printStackTrace();
        }
    }


    @Test
    public void naiveGravityTest(){
        var a = ArmDynamics.calcGravityTorques(new double[]{Math.PI/2,0,0});
        System.out.println(a);
        assertTrue(a.isIdentical(new SimpleMatrix(3,1), 0.1));
    }

    @Test
    public void kinematicsTest0(){
        double[] jv = {2,3,1};
        double[] cv = ArmDynamics.forwardKinematics(jv);
        // System.out.println(Arrays.toString(ArmDynamics.inverseKinematics(cv)));
        assertArrayEquals(jv, ArmDynamics.inverseKinematics(cv), 0.01);
    }

    @Test
    public void kinematicsTest1(){
        double[] jv = {3.1,0.1,2};
        double[] cv = ArmDynamics.forwardKinematics(jv);
        // System.out.println(Arrays.toString(ArmDynamics.inverseKinematics(cv)));
        assertArrayEquals(jv, ArmDynamics.inverseKinematics(cv), 0.01);
    }

    @Test
    public void kinematicsTest2(){
        double[] jv = {-2,0.1,-1};
        double[] cv = ArmDynamics.forwardKinematics(jv);
        // System.out.println(Arrays.toString(ArmDynamics.inverseKinematics(cv)));
        assertArrayEquals(jv, ArmDynamics.inverseKinematics(cv), 0.01);
    }

    @Test
    public void forwardKinematicsTest(){
        double[] jv = {0.5,1.6,0.7};
        double[] expected = {-5.126, 37.011, 2.8};
        double[] result = ArmDynamics.forwardKinematics(jv);
        // System.out.println(Arrays.toString(result));
        assertArrayEquals(expected, result, 0.01);
    }
}