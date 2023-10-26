package frc.robot.autoframe;

import frc.robot.Scoring.ScoringTier;
import frc.robot.utils.Piece;
import frc.robot.Scoring;
import frc.robot.subsystems.swerve.SwerveInstruction;
import static frc.robot.Auto.movement;
import static frc.robot.Auto.rotSpeed;

public class Score extends AutoFrame {
    private int nodeCol;
    private ScoringTier tier;
    private Piece piece;

    /**
     * Blocking. Instructs the robot to score at a particular node column and tier with a given piece. Bundles operations
     * for robot movement, arm movement, and scoring into one AutoFrame. 
     * @param nodeCol the column to move at, ranging 0 to 8, starting with index 0 for the column closest to the wall.
     * @param tier the ScoringTier on which the robot should score.
     * @param piece the Piece (Cone or Cube) that the robot should score with. 
     */
    public Score(int nodeCol, ScoringTier tier, Piece piece){
        this.blocking = true;
        this.nodeCol = nodeCol;
        this.tier = tier;
        this.piece = piece;
    }

    @Override
    public void start(){
        Scoring.setScoringTarget(nodeCol, tier, piece, 1.5);
    }

    @Override
    public void update(){
        SwerveInstruction si = Scoring.update();
        movement = si.movement;
        rotSpeed = si.rotation;
        this.done = !Scoring.isScoring();
    }
}