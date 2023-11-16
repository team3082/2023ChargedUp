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
        var m1 = ArmController.calculateMassMatrix(jv);
        var m2 = ArmController.calculateMassMatrix2(jv);
        System.out.println(m1);
        System.out.println(m2);
        assertTrue(m1.isIdentical(m2, 1));
        }catch(RuntimeException e){
            e.printStackTrace();
        }
    }

    @Test
    public void massMatrixBenchmark(){
        double[] jv = {0.000146471,0.002546542,3.45743};
        int start = (int) System.currentTimeMillis();
        for(int i = 0; i < 1000; i++){
            jv[0] = (jv[0] * 2394) % 3;
            jv[1] = (jv[1] * 2494) % 3;
            jv[2] = (jv[2] * 2334) % 3;
            ArmController.calculateMassMatrix2(jv);
        }
        System.out.println((int) System.currentTimeMillis() - start);
    }
}
