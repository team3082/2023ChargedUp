package frc.robot.autoframe;

import static frc.robot.Auto.rotSpeed;
import static frc.robot.Auto.inchOff;

import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Telemetry;

// ROTATES THE ROBOT UNTIL IT HITS A PIECE

public class ScanForPiece extends AutoFrame {

    private final double min, max;
    private boolean CW;
    private double lastDist = Double.MAX_VALUE;

    public ScanForPiece(double min, double max) {
        this.blocking = true;
        this.min = min;
        this.max = max;
    }

    @Override
    public void start() {
        this.CW = Pigeon.getRotationRad()> Math.PI/2;
    }

    @Override
    public void update(){
        double currentDist;
        try {
            currentDist = Telemetry.getProximity();
        } catch (Exception e){
            currentDist = Double.MAX_VALUE;
        }

        if(currentDist < 100) {
            // Rotate slowly after we see a target until the distance starts increasing again so that we center ourselves
            // on the middle of the piece instead of the edge of the piece
            if (currentDist > lastDist){
                inchOff = currentDist;
                this.done = true;
                return;
            }
            // rotSpeed = 0.5 * (this.CW?1:-1);
            rotSpeed = 0.03 * (this.CW ? 1 : -1);
            lastDist = currentDist;
        } else {
            this.CW = this.CW ? !(Pigeon.getRotationRad() > max) : (Pigeon.getRotationRad() < min);
            rotSpeed = 0.075 * (this.CW ? 1 : -1);
        }
    }

}
