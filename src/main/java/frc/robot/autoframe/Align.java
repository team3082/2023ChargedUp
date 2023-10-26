package frc.robot.autoframe;

import frc.robot.Scoring;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.utils.Vector2;

import static frc.robot.Auto.movement;
import static frc.robot.Auto.rotSpeed;

public class Align extends AutoFrame {
    
    private int column;

    public Align(int col) {
        column = col;
    }

    @Override
    public void start(){
        SwervePID.setDestPt(Scoring.getScoringTarget(column).add(new Vector2(0.8, 7))); // this makes align work idk why   
        SwervePID.setDestRot(3.0 * Math.PI / 2.0);
    }

    @Override
    public void update(){
        double angleDeg = Vision.limelight.getXOffset();
        double velX = column%3==1 ? SwervePID.updateOutputX() : angleDeg * -0.015;

        movement = new Vector2(velX, SwervePID.updateOutputY());
        rotSpeed = SwervePID.updateOutputRot();
    
        done = movement.mag() < 0.001;
    }

}
