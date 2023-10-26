package frc.robot.autoframe;

import frc.robot.Scoring;


public class MoveToNode extends MoveTo {
    /**
     * Extends the MoveTo frame, moving to a specified node column rather than a (x, y) coordinate
     * @param nodeCol the column to move to, ranging 0 to 8, starting with index 0 for the column closest to the wall.
     * closest to the wall. 
     */
    public MoveToNode(int nodeCol){
        super(Scoring.getScoringTarget(nodeCol));
    }
}
