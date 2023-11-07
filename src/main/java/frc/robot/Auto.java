package frc.robot;

import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.utils.Piece;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;

import com.ctre.phoenixpro.controls.NeutralOut;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Scoring.ScoringTier;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Telemetry.Severity;
import frc.robot.subsystems.swerve.*;

import frc.robot.autoframe.*;

public class Auto {

    public static int targetCol;
    public static ArmPosition endingArmPosition = null;

    public static double inchOff;

    public static double moveScale = 1;
    public static double rotScale = 1;

    private static double grabOffset = 18; // Originally 18

    public static Piece scoringPiece = Piece.CONE;

    public static double startTimestamp = 0;

    // CUSTOM AUTOS ================================================

    // PID test
    // Robot moves forward 10 inches
    public static void test(){
        Vector2 pos = SwervePosition.getPosition();
        AutoFrame[] frames = new AutoFrame[] {
            new SetVision(false), // we want accurate odometry throughout
            new MoveTo(new Vector2(pos.x, pos.y + 10))
        };
        queueFrames(frames);
    }

    public static void scoreTesting(){
        AutoFrame[] frames= new AutoFrame[] {
            new Recalibrate(true, true, 0.5),
            new Score(5, ScoringTier.TOP, Piece.CONE)
        };
        queueFrames(frames);
    }

    public static void testPID(int targetColumn) {
        AutoFrame[] frames = new AutoFrame[] {
                new RotateToGrid(),
                new MoveToNode(targetColumn),
                new WaitUntil(()-> {return RTime.now()>10;})
                // new MoveArm(targetTier),
        };
        queueFrames(frames);
    }

    // Autonomous
    public static void score(int targetColumn, ScoringTier tier, Piece piece) {
        AutoFrame[] frames = new AutoFrame[] {
            //new Recalibrate(true, false, 0.5),
            new RotateToGrid(),
            new Score(targetColumn, tier, piece),
            //new WaitToCompletion(MoveToNode.class),
            //new ,
            //new WaitToCompletion(MoveArm.class),
        };
        queueFrames(frames);
    }

    public static void balanceAndScore(){
        AutoFrame[] frames = new AutoFrame[] {
            // new SetOdometryPos(scoringPiece==Piece.CUBE?null:Scoring.getScoringTarget(5)),
            new SetGrabMode(scoringPiece),
            //new Recalibrate(true, false, 0.25),
            new SetVision(true),
            (scoringPiece==Piece.CUBE?
                new Score(4, ScoringTier.TOP, Piece.CUBE) :
                new Score(3, ScoringTier.TOP, Piece.CONE) ) ,
            new SetVision(false),

            new MoveArmIntermediate(ArmPosition.PRIMED, ArmPosition.STARTING),
            new Rotate(Math.PI * 3 / 2 + Math.PI / 12),
            new SetMoveScale(1.2),
            new MoveTo(
                (scoringPiece==Piece.CUBE?Scoring.getScoringTarget(4):Scoring.getScoringTarget(3))
                .add(new Vector2(0, 10))), // Move back bc new armPIDs
            // new WaitUntil(() -> {return Arm.atPosition();}),
            new WaitUntil(() -> {return Arm.getWristPos().x<22;}),
            new ClearActive(),
            // new WaitToCompletion(MoveArm.class),
            new MoveTo(49.61, 0),
            // new WaitToCompletion(MoveTo.class),
            // new Recalibrate(1),
            // new Rotate(Math.PI/2),
            // new MoveTo(49.61, -200),
            // new WaitToCompletion(MoveTo.class),
            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) > Math.toRadians(10.0);}),
            // new Wait(0.8),
            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) < Math.toRadians(2);}),
            
            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) > Math.toRadians(10.0);}),

            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) < Math.toRadians(2);}),

            new ClearActive(),
            new SetMoveScale(1),
            new Rotate(Math.PI * 3 / 2 + Math.PI / 12),
            new Wait(0.5),
            new ClearActive(),
            new MoveTo(49.61, -300),
            
            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) > Math.toRadians(10.0);}),
            new Wait(0.8),
            new ClearActive(),
            new Balance()
        };
        endingArmPosition = ArmPosition.PRIMED;
        queueFrames(frames);
    }

    public static void balanceAndScoreMobility() {
        AutoFrame[] frames = new AutoFrame[] {
            // new SetOdometryPos(scoringPiece==Piece.CUBE?null:Scoring.getScoringTarget(5)),
            new SetGrabMode(scoringPiece),
            //new Recalibrate(true, false, 0.25),
            (scoringPiece==Piece.CUBE?
                new Score(4, ScoringTier.TOP, Piece.CUBE) :
                new Score(3, ScoringTier.TOP, Piece.CONE) ) ,
            
            new MoveArmIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.PRIMED),
            new SetMoveScale(1),
            new MoveTo(49.61, 0),
            new WaitToCompletion(MoveArmIntermediate.class),
            new SetMoveScale(1),

            // Wait until we're climbing the charge station
            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) > Math.toRadians(10.0);}),
            // Wait until we're (more or less) level on the charge station
            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) < Math.toRadians(4.0);}),  // prev 2   
            // Wait to descend the charge station
            new Wait(0.5),      
            // Wait until we're level on the ground
            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) < Math.toRadians(2.0);}),

            new ClearActive(),
            new Wait(0.5),
            new MoveTo(49.61, -300),
            
            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) > Math.toRadians(10.0);}),
            new Wait(0.75),
            new ClearActive(),
            new Balance()
        };
        endingArmPosition = ArmPosition.STARTING;
        queueFrames(frames);
    }

    public static void substationAuto() {
        AutoFrame[] frames = new AutoFrame[] {
            new SetGrabMode(scoringPiece),
            new SetVision(true),
            (scoringPiece==Piece.CUBE?
                new Score(7, ScoringTier.TOP, Piece.CUBE) :
                new Score(8, ScoringTier.TOP, Piece.CONE) ) ,

            new Wait(0.1),
            new MoveTo(
                    (scoringPiece==Piece.CUBE?Scoring.getScoringTarget(7):Scoring.getScoringTarget(8))
                    .add(new Vector2(0, 10))), // Move back bc new armPIDs\
            new MoveArmIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.PRIMED),
            new WaitUntil(() -> { return Arm.atPosition(); }),
            new ClearActive(),
            new SetRotScale(1.0),
            new SetMoveScale(1.0),
            new MoveTo(-22.39,-47-grabOffset-10),
            //new Rotate(Math.PI/2.0),
            new WaitToDist(SwervePID.yPID, 60.),
            new Rotate(Math.PI / 2.0),
            new WaitToCompletion(MoveTo.class),
            new SetGrabMode(Piece.CUBE),
        };
        endingArmPosition = ArmPosition.PRIMED;
        queueFrames(frames);
    }

    public static void substation_J_Auto() {
        AutoFrame[] frames = new AutoFrame[] {
            // new SetOdometryPos(scoringPiece==Piece.CUBE?null:Scoring.getScoringTarget(8)),
            new SetGrabMode(scoringPiece),
            //new Recalibrate(true, false, 0.25),
            (scoringPiece==Piece.CUBE?
                new Score(7, ScoringTier.TOP, Piece.CUBE) :
                new Score(8, ScoringTier.TOP, Piece.CONE) ) ,
            new SetMoveScale(0.25),
            new MoveTo(-25,-90),
            new WaitUntil(() -> { return Arm.atPosition(); }),
            new MoveArm(ArmPosition.STARTING),
            new SetMoveScale(1),
            //new MoveArm(ArmPosition.STARTING),
            new WaitToDist(SwervePID.yPID, 12),
            new ClearActive(),
            new MoveTo(62,-100),
            new WaitToDist(SwervePID.xPID, 12),
            new ClearActive(),
            new MoveTo(50, -300),

            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) > Math.toRadians(10.0);}),
            new Wait(1),

            new ClearActive(),
            new Balance()

        };
        endingArmPosition = ArmPosition.GROUND;
        queueFrames(frames);
    }

    public static void edge_J_Manuver() {
        AutoFrame[] frames = new AutoFrame[] {
            // new SetOdometryPos(scoringPiece==Piece.CUBE?null:Scoring.getScoringTarget(0)),
            new SetGrabMode(scoringPiece),
            //new Recalibrate(true, false, 0.25),
            (scoringPiece==Piece.CUBE?
                new Score(1, ScoringTier.TOP, Piece.CUBE) :
                new Score(0, ScoringTier.TOP, Piece.CONE) ) ,
            new SetMoveScale(0.25),
            new MoveTo(125,-90),
            new WaitUntil(() -> { return Arm.atPosition(); }),
            new MoveArm(ArmPosition.STARTING),
            new SetMoveScale(1),
            //new MoveArm(ArmPosition.STARTING),
            new WaitToDist(SwervePID.yPID, 12),
            new ClearActive(),
            new MoveTo(38,-100),
            new WaitToDist(SwervePID.xPID, 12),
            new ClearActive(),
            new MoveTo(50, -300),

            new WaitUntil(() -> {return Math.abs(Pigeon.getKHat().getPitch()-Math.PI/2) > Math.toRadians(10.0);}),
            new Wait(1),

            new ClearActive(),
            new Balance()

        };
        endingArmPosition = ArmPosition.GROUND;
        queueFrames(frames);
    }

    public static void edgeAuto() {
        AutoFrame[] frames = new AutoFrame[] {
            // new SetOdometryPos(scoringPiece==Piece.CUBE?null:Scoring.getScoringTarget(0)),
            new SetGrabMode(scoringPiece),
            //new Recalibrate(true, false, 0.25),
            (scoringPiece==Piece.CUBE?
                new Score(1, ScoringTier.TOP, Piece.CUBE) :
                new Score(0, ScoringTier.TOP, Piece.CONE) ) ,
            new MoveArmIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.PRIMED),
            new MoveTo(
                (scoringPiece==Piece.CUBE?Scoring.getScoringTarget(1):Scoring.getScoringTarget(0))
                .add(new Vector2(0, 10))), // Move back bc new armPIDs
            new WaitUntil(() -> { return Arm.atPosition(); }),
            new ClearActive(),
            // new MoveTo(121.61,-120.36),
            // new WaitToCompletion(MoveTo.class),
            // new Recalibrate(1),
            new MoveTo(121.61, -47-grabOffset-10),

            //new Rotate(Math.PI/2),
//             new Rotate(Math.PI/2.0),
            new WaitToDist(SwervePID.yPID, 5),
            // new SetGrabMode(Piece.CONE),
            //new MoveArmIntermediate(ArmPosition.GROUND_INTERMEDIATE, ArmPosition.GROUND),
            new Rotate(Math.PI/2),
            //new SetGrabMode(Piece.CONE),
        };
        endingArmPosition = ArmPosition.PRIMED;
        queueFrames(frames);
    }

    public static void edgeAutoWithDistanceSensor() {
        AutoFrame[] frames = new AutoFrame[] {
            // new SetOdometryPos(scoringPiece==Piece.CUBE?null:Scoring.getScoringTarget(0)),
            new SetGrabMode(scoringPiece),
            new SetVision(true),
            //new Recalibrate(true, false, 0.5),
            (scoringPiece==Piece.CUBE?
                new Score(1, ScoringTier.TOP, Piece.CUBE) :
                new Score(0, ScoringTier.TOP, Piece.CONE) ) ,
            new MoveArmIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.GROUND),
            // new MoveTo(
            //     (scoringPiece==Piece.CUBE?Scoring.getScoringTarget(1):Scoring.getScoringTarget(0))
            //     .add(new Vector2(0, 10))), // Move back bc new armPIDs

            new SetGrabMode(Piece.CUBE),

            new SetRotScale(0.5),
            new SetMoveScale(1.35),

            new MoveTo(121.61, -47-grabOffset),
            new Rotate(Math.PI/2 + Math.toRadians(45)), // Set up for sweep

            new WaitUntil(() -> { return Arm.getWristPos().y <22; }),
            new SetMoveScale(1),
            new WaitToCompletion(MoveTo.class),
            // new WaitToCompletion(Rotate.class),

            // new MoveArm(ArmPosition.GROUND_INTERMEDIATE),
            // NIIIIIIIIIICK AND HIS EXCEPTIONS
            // please fix
            new ClearActive(),
            new SetRotScale(1),
            
            new SetMoveScale(0),
            
            new Rotate( () -> { return Pigeon.getRotationRad(); }),
            new MoveTo( () -> {
                Vector2 move;
                // try {
                    move = Vector2.fromPolar(Pigeon.getRotationRad(), inchOff);
                    move.x *= (DriverStation.getAlliance() == Alliance.Red) ? -1 : 1;
                    return SwervePosition.getPosition().add(move);
                // } catch (Exception e) {
                //     move = Vector2.fromPolar(Pigeon.getRotationRad(), 60 - grabOffset+6);
                //     move.x *= (DriverStation.getAlliance() == Alliance.Red) ? -1 : 1;
                //     return SwervePosition.getPosition();
                // }
            }),
            new WaitToCompletion(MoveArmIntermediate.class),
            new SetMoveScale(1),
            new Intake(),

            new WaitToCompletion(MoveTo.class),

            new SetGrabMode(scoringPiece==Piece.CUBE?Piece.CONE:Piece.CUBE),
            new Wait(0.25),

            new SetVision(true),
            (scoringPiece==Piece.CUBE? // SCORE OPPOSITE PIECE
                new Score(0, ScoringTier.TOP, Piece.CONE) :
                new Score(1, ScoringTier.TOP, Piece.CUBE) )

        };
        queueFrames(frames);
    }

    public static void middleAuto() {
        AutoFrame[] frames = new AutoFrame[] {
            new SetGrabMode(scoringPiece),
            // new SetVision(true),
            //new Recalibrate(true, false, 0.25),
            (scoringPiece==Piece.CUBE?
                new Score(4, ScoringTier.TOP, Piece.CUBE) :
                new Score(3, ScoringTier.TOP, Piece.CONE) ) ,
            new MoveTo(
                (scoringPiece==Piece.CUBE?Scoring.getScoringTarget(4):Scoring.getScoringTarget(3))
                .add(new Vector2(0, 3))), // Move back bc new armPIDs
            new MoveArmIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.PRIMED),
        };
        queueFrames(frames);
    }

    public static void doNothing() {
        queueFrames(new AutoFrame[]{});
    }

    // Teleauto
    public static void balance() {
        AutoFrame[] frames = new AutoFrame[] {
            new Balance()
        };
        queueFrames(frames);
    }

    // Disabled
    public static void safeDisable() {
        AutoFrame[] frames = new AutoFrame[] {
                new MoveArm(ArmPosition.PRIMED),
                new WaitToCompletion(MoveArm.class),
                new MoveArm(ArmPosition.STARTING),
        };
        queueFrames(frames);
    }

    
    // AUTO SYSTEM =================================================
    public static Queue<AutoFrame> queuedFrames;
    public static HashSet<AutoFrame> activeFrames = new HashSet<>();

    public static double rotSpeed;
    public static Vector2 movement;

    private static void queueFrames(AutoFrame[] frames) {
        activeFrames.clear();
        queuedFrames = new LinkedList<>(Arrays.asList(frames));
        startTimestamp = RTime.now();
    }

    public static void update() {
        rotSpeed = 0;
        movement = new Vector2();

        boolean advanceFrame = true;
        HashSet<AutoFrame> finishedFrames = new HashSet<>();
        for (AutoFrame frame : activeFrames) {

            // For debugging, print the classes of all of the active frames
            // System.out.print(frame.getClass().getSimpleName() + " ");
            Telemetry.log(Telemetry.Severity.INFO, frame.getClass().getSimpleName() + " ");

            // Update and/or finish the frame according to whether or not its "done" boolean has been set to true
            frame.update();
            if (frame.done){
                finishedFrames.add(frame);
                frame.finish();
            }

            // If there's any blocking frame active that didn't just end, don't advance to the next frame
            else if (frame.blocking)
                advanceFrame = false;
        }
        // Remove all of the finished frames from the set of active frames
        activeFrames.removeAll(finishedFrames);

        System.out.println(); // Newline for debugging readability

        // Conditions for advancing the frame:
        // AdvanceFrame must be true, meaning there are no active, blocking frames
        // QueuedFrames must have items remaining
        if (advanceFrame && !queuedFrames.isEmpty()) {
            AutoFrame newFrame = queuedFrames.remove();
            newFrame.start();
            activeFrames.add(newFrame);
        }

        // Print "Done" to signify Auto has finished
        if(queuedFrames.isEmpty() && activeFrames.isEmpty()){
            Telemetry.log(Telemetry.Severity.INFO, "Auto complete.");
            Telemetry.log(Severity.DEBUG, "finished in " + (RTime.now() - startTimestamp));
        }

        // Rotate and drive the robot according to the output of the active AutoFrames
        SwerveManager.rotateAndDrive(rotSpeed * rotScale, movement.mul(moveScale));
    }

    public static void experimentalSubstation(){
        // Scores closest to substation mid, goes to pickup a cone middle, returns, and yeets the last cone into low
        AutoFrame[] frames = {
            new SetVision(true),
            new SetGrabMode(Piece.CONE),
            new MoveArm(ArmPosition.PRIMED),
            new WaitToCompletion(MoveArm.class),
            // Arm has reached primed and is now moving to SCORE_INTERMEDIATE, thus limelight can now see so we safe to align
            new ClearActive(),
            new MoveArmIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.CONE_MIDDLE),
            new Align(6),
            new WaitToCompletion(Align.class),
            new WaitToCompletion(MoveArmIntermediate.class),
            new SetGrabMode(Piece.CUBE),
            new Wait(0.2),
            // We have scored, now drive towards mid while setting up for a scan (rotating and moving arm into position)
            new MoveArmIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.GROUND),
            //new SetRotScale(1), // Slightly slower as arm is pretty extended rn
            SetMoveScale.setAbsoluteMoveScale(0.5), // MAX SPEED!!!
            //new Rotate(() -> {return DriverStation.getAlliance()==Alliance.Red? Math.PI/3.0 : 2.0*Math.PI / 3.0; }), // Rotate towards substation, not charge station
            new MoveTo(-22.4, -47 - 60), // Get into position for scan. Center of robot theoretically 30 inches away from cone
            
            new WaitToDist(SwervePID.yPID, 24), // Last 2 ft use normal speed
            new SetMoveScale(1), // Back to 0.3

            // Wait for distance sensor in position & Robot in position            
            new WaitToCompletion(MoveTo.class),
            new Rotate(Math.PI/2),
            new WaitToCompletion(MoveArmIntermediate.class),
            new WaitToCompletion(Rotate.class),
            new ClearActive(), // Clear rotate just in case
            new SetRotScale(1), // Scan needs normal rotation scale
            //new ScanForPiece(Math.PI / 3.0, 2.0 * Math.PI / 3.0),
            //new Rotate(() -> { return Pigeon.getRotationRad(); }),
            

            // new MoveTo(() -> { // Move to the spot
            //     Vector2 move;
            //     move = Vector2.fromPolar(Pigeon.getRotationRad(), inchOff);
            //     move.x *= (DriverStation.getAlliance() == Alliance.Red) ? -1 : 1;
            //     return SwervePosition.getPosition().add(move);
            // }),
            new MoveTo(-22.4, -47 -30),
            new WaitToDist(SwervePID.yPID, 2),
            //new Wait(0.5),
            new SetGrabMode(Piece.CONE),
            new ClearActive(), // Clear rotate
            new MoveArm(ArmPosition.PRIMED),
            new Rotate(3.0*Math.PI/2.0),
           // new SetMoveScale(1.0), //MAX SPEED!!!
            SetMoveScale.setAbsoluteMoveScale(1.0), // ACTUAL MAX SPEEEEEEEED!!!
            new MoveToNode(8),
            new WaitUntil(() -> { return Vision.hasTarget(); }), // We wanna have accurate odometry before yeeting
            new WaitToDist(SwervePID.yPID, 60), // Robot center just inside charge station
            new WaitToDist(SwervePID.xPID, 10), // +/- 10 in
            new Outtake(), // YEET!!
            new Wait(0.25),
            new ClearActive(),
            //NeutralOut(),
        };

        queueFrames(frames);
    }

    public void funnyAuto(){
        AutoFrame[] frames = {
            new WaitUntil(() -> { SwervePosition.setPosition(new Vector2()); return true; }),
            new MoveTo(0,100),
        };

        queueFrames(frames);
    }

    public static void experimentalEdge(){
        
    }

}