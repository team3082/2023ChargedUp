package frc.robot.subsystems.arm;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.simple.*;
import org.junit.jupiter.api.Test;

public class ArmControllerTest {
    @Test
    public void ejmlTest(){
        SimpleMatrix m = new SimpleMatrix(3,3, true, new double[]{1.0,1.0,1.0,0,0,0,0,0,0});
        SimpleMatrix rot = new SimpleMatrix(3,3,true, new double[]{0,-1,0,1,0,0,0,0,1});
        SimpleMatrix in = new SimpleMatrix(3,1,false, new double[]{2.0,3,2});
        SimpleMatrix exp = new SimpleMatrix(3,1, false, new double[]{0,7,0});

        var result = rot.mult(m).mult(in);
        System.out.println(result);
        System.out.println(exp);
        assertTrue(result.isIdentical(exp, 0.1));
    }

    @Test
    public void massMatrixTest(){
        double[] jv = {2,1,2};
        try{
        var m1 = ArmController.calcMassMatrix(jv);
        var m2 = ArmController.calcMassMatrix2(jv);
        System.out.println(m1);
        System.out.println(m2);
        assertTrue(m1.isIdentical(m2, 1));
        }catch(RuntimeException e){
            e.printStackTrace();
        }
    }

    @Test
    public void transformationTest(){
        double[] jv = {Math.PI/2, Math.PI/2,Math.PI/2};
        SimpleMatrix input = new SimpleMatrix(4,1,false,new double[]{0,0,0,1});
        SimpleMatrix transform = ArmController.calcForwardMatrixb2(jv);
        SimpleMatrix expected = new SimpleMatrix(4,1,false, new double[]{-26,10,3*Math.PI/2,1});

        var result = transform.mult(input);
        System.out.println(result);
        System.out.println(expected);
        assertTrue(result.isIdentical(expected, 0.1));
    }

    @Test
    public void naiveGravityTest(){
        var a = ArmController.calcGravityTorques(new double[]{Math.PI/2,0,0});
        System.out.println(a);
        assertTrue(a.isIdentical(new SimpleMatrix(3,1), 0.1));
    }
}
