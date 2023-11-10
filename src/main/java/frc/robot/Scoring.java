package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm.ArmControlMode;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.swerve.SwerveInstruction;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.utils.Piece;

public class Scoring {

    public enum ScoringTier{
        TOP, MIDDLE, BOTTOM
    }

    /** 
     * Use this enum to keep track of where the robot currently is in the scoring process
     */
    public enum Progress{
        /**
         * The robot is at the scoring position, moving the arm to score
         */
        AT_NODE, 
        /**
         * The robot has the arm at the right position, and is placing the piece
         */
        MOVING_FORWARD,
        /**
         * Robot is aligning to the cone node.
         */
        // ALIGN,
        /**
         * Robot is scoring
         */
        SCORING, 
        /**
         * The robot has obtained the piece and is driving to the scoring position. Lasts one frame
         */
        IN_TRANSIT, 
        /**
         * The robot hasn't been ordered to score
         */
        NOT_SCORING,
        /**
         * The robot is preparing to drive to the scoring position. This lasts one frame
         */
        PREP;
    }

    /*
     * (Scoring Positions in Blue Community)
     * 0 - (137.61, -252.88)
     * 1 - (115.61, -252.88)
     * 2 - (93.61, -252.88)
     * 3 - (71.61, -252.88)
     * 4 - (49.61, -252.88)
     * 5 - (27.61, -252.88)
     * 6 - (5.61, -252.88)
     * 7 - (-16.61, -252.88)
     * 8 - (-38.61, -252.88)
     */

    
    public static Progress currentProgress;
    public static Piece heldPiece;
    public static ScoringTier endTier;
    //Just used for progress updates
    private static Vector2 scoringPos;
    public static double stopTime;

    public static double yBuffer;
    private static double scoringY;

    private static int column;

    private static double moveTimeoutDuration;
    private static double moveTimeoutTime;


    public static void init(){
        currentProgress = Progress.NOT_SCORING;
        //                             14.25 is the length of the ground dividers (p. 27 of manual)
        // -285.16 + 14.25 + 3.25 + 15
        // scoringY = -252.66;
        scoringY = RobotConfig.gridY + 14.25 + RobotConfig.bumperThickness + (RobotConfig.frameLength/2);
        yBuffer = RobotConfig.yBuffer;
    }
    

    public static void stop(){
        currentProgress = Progress.NOT_SCORING;
        Arm.setState(ArmControlMode.NEUTRAL, null, 0);
    }

    /**
     * Gets the position the robot should move to in order to score at a given column.
     * @param col The column of the scoring node on (0,8).
     *            0 is the farthest column from the right from the perspective of the driver's station
     * @return A Vector2 representing the position we should score at
     */
    public static Vector2 getScoringTarget(int col) {
        return new Vector2((22*8-38.61) - (col * 22), scoringY);
    }

    /** 
     * Sets the target scoring node.
     * @param col The column of the scoring node on (0,8).
     *            0 is the farthest column from the right from the perspective of the driver's station
     * 
     * @param tier The level to place the piece on. BOTTOM is floor level, MIDDLE is middle level,
     *             and TOP is the uppermost level
     */
    public static void setScoringTarget(int col, ScoringTier tier, Piece piece, double moveTimeoutSeconds){
        column = col;
        scoringPos = getScoringTarget(col);
        heldPiece = piece;
        endTier = tier;
        currentProgress = Progress.PREP;
        moveTimeoutDuration = moveTimeoutSeconds;
    }

    // public static void setArmPos(int armPos) {
        // ArmPosition pos = positions[armPos];
        // Arm.setState(pos);
    // }

    // public static void update() {
    //     if (!armIsMoving) {
    //         if (atLocation) {
    //             SwerveManager.rotateAndDrive(0, new Vector2());
    //             //setArmPos(0);
    //             armIsMoving = true;
    //             return;
    //         }
    //         // SwervePID.setDestPt(scoringPos);
    //         SwerveManager.rotateAndDrive(SwervePID.updateOutputRot() * .5, SwervePID.updateOutputVel());
    //         atLocation = SwervePID.atDest() && SwervePID.atRot();
    //     } else {
    //         isScoring = false;
    //         atArmPos = true;
    //         //if (Arm.atPosition()){
    //         //    atArmPos = true;
    //         //    isScoring = false;
    //         //}
    //     }
    // }

    private static void setTargetArmState(){
        switch(heldPiece){
            case CONE:
                switch(endTier){
                    case TOP:
                        Arm.setStateIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.CONE_TOP);
                        break;
                    case MIDDLE:
                        Arm.setState(ArmPosition.CONE_MIDDLE);
                        break;
                    case BOTTOM:
                        Arm.setState(ArmPosition.CONE_BOTTOM);
                        break;
                }
                break;
            case CUBE:
                switch(endTier){
                    case TOP:
                        Arm.setStateIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.CUBE_TOP);
                        break;
                    case MIDDLE:
                        Arm.setState(ArmPosition.CUBE_MIDDLE);
                        break;
                    case BOTTOM:
                        Arm.setState(ArmPosition.CUBE_BOTTOM);
                        break;
                }
                break;
        }
    }

    public static SwerveInstruction update() {
        Telemetry.log(Telemetry.Severity.INFO, "" + currentProgress);
        Telemetry.log(Telemetry.Severity.INFO, SwervePID.atDest() + ", " + Arm.atPosition());
        if(heldPiece==Piece.CUBE)
            Manipulator.setCubeMode();
        switch (currentProgress){
            case NOT_SCORING:
                //does nothing 
                break;
            case PREP:
                //sets the robots arm and position destinations. Lasts one frame
                armToPrimed();
                SwervePID.setDestState(scoringPos.add(new Vector2(0, yBuffer)), 3 * Math.PI / 2);
                currentProgress = Progress.IN_TRANSIT;
                break;
            case IN_TRANSIT:
                //drives to scoringPos
                if(SwervePosition.getPosition().sub(SwervePID.getDest()).mag()<5 && SwervePID.rotPID.getError()<Math.toRadians(2)){
                    currentProgress = Progress.AT_NODE;
                    setTargetArmState();
                } else {
                    return SwervePID.updateAll();
                }
                break;
            case AT_NODE:
                //moves arm to target arm position
                if(Arm.atPosition()){
                    moveTimeoutTime = RTime.now() + moveTimeoutDuration;
                    SwervePID.setDestPt(scoringPos.add(new Vector2(0.8, 7))); // this makes align work idk why
                    currentProgress = Progress.MOVING_FORWARD;
                }
                return SwervePID.updateAll();
            case MOVING_FORWARD:
                double angleDeg = Vision.limelight.getXOffset();
                double velX = heldPiece == Piece.CUBE ? SwervePID.updateOutputX() : angleDeg * -0.015;

                Vector2 movement = new Vector2(velX, SwervePID.updateOutputY());
                double rotation = SwervePID.updateOutputRot();
            
                if(movement.mag() < 0.001 || RTime.now() > moveTimeoutTime){
                    stopTime = RTime.now() + RobotConfig.scoringWaitTime;
                    currentProgress = Progress.SCORING;
                }

                return new SwerveInstruction(rotation, movement);
            // case ALIGN: 
            //     double angleDeg = Vision.limelight.getXOffset();
            //     double vel = angleDeg * -0.015;
            //     if (Math.abs(vel) < 0.0010)
            //         currentProgress = Progress.SCORING;
            //     return new SwerveInstruction(0, new Vector2(vel, 0));
            case SCORING:
                if(column%3==1){
                    Manipulator.outtake();
                } else {
                    Manipulator.setCubeMode();
                }
                if(RTime.now() > stopTime){
                    // armToPrimed();  
                    if(column%3==1){
                        Manipulator.off();
                    } else {
                        Manipulator.setConeMode();
                    }
                    currentProgress = Progress.NOT_SCORING;
                }
                break;
        }
        return new SwerveInstruction(); // We are not "IN_TRANSIT" so no movement
    }

    private static void armToPrimed(){
        switch(heldPiece){
            case CUBE:
                Arm.setState(ArmPosition.PRIMED);
                break;
            case CONE: 
                Arm.setState(ArmPosition.PRIMED);
        }
    }


    /**
     * Whether the robot is currently scoring
     * @return  if the robot is scoring or not
     */
    public static boolean isScoring(){
        return currentProgress != Progress.NOT_SCORING;
    }
}
