package frc.robot.utils;

public class Matrix3 {
    public double[] data;
    
    // ROW-MAJOR data stored like this:
    // * -> * -> *
    // 🡗---------┘
    // * -> * -> *
    // 🡗---------┘
    // * -> * -> * 

    public Matrix3(double[] data) {
        if (data.length != 9)
            throw new Error("Input data to Matrix3 constructor must be a double[] of length 9");
        this.data = data;
    }

    /**
    * Right-multiply this matrix to a Vector2
    * @param rhs the vector to multiply by
    */
    public Vector2 mul2(Vector2 rhs){
        // TALK TO DEVIN
        double[] v = {rhs.x, rhs.y, 1.0};
        return new Vector2(
            data[0] * v[0] + data[1] * v[1] + data[2] * v[2],
            data[3] * v[0] + data[4] * v[1] + data[5] * v[2]
        );
    }
}