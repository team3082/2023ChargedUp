package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.controllermaps.LogitechF310;
import frc.robot.Auto;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.Scoring;
import frc.robot.subsystems.Arm.ArmControlMode;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Telemetry.Severity;
import frc.robot.subsystems.swerve.SwerveInstruction;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.Piece;
import frc.robot.utils.RMath;
import frc.robot.utils.Vector2;

public class OI {
    
    public static Joystick driverStick;
    public static Joystick buttonBoard;
    public static Joystick flightStick;

    static final int moveX = LogitechF310.AXIS_LEFT_X;
    static final int moveY = LogitechF310.AXIS_LEFT_Y;
    static final int rotateX = LogitechF310.AXIS_RIGHT_X;
    static final int boost = LogitechF310.AXIS_RIGHT_TRIGGER;
    static final int pigeonZero = LogitechF310.BUTTON_Y;
    static final int driveToNode = LogitechF310.BUTTON_A;
    static final int parkingBrake = LogitechF310.BUTTON_B;
    public static final int balance = LogitechF310.BUTTON_X;
    //TODO noah chooses keybind. currently a toggle, but feel free to change
    

    // static final int armX = 100;
    // static final int armY = 100;
    static final int fineRotateX = LogitechF310.AXIS_RIGHT_X;
    static final int modeSwitch = LogitechF310.BUTTON_LEFT_BUMPER;
    static final int groundPickup = LogitechF310.BUTTON_RIGHT_BUMPER;
    static final int groundScoring = LogitechF310.BUTTON_X;
    static final int midScoring = LogitechF310.BUTTON_A;
    static final int topScoring = LogitechF310.BUTTON_B;
    static final int armDisable = LogitechF310.BUTTON_BACK;
    static final int 
    primed = LogitechF310.BUTTON_Y;
    static final int releasePiece = LogitechF310.AXIS_LEFT_TRIGGER;
    static final int grabPiece = LogitechF310.AXIS_RIGHT_TRIGGER;
    static final int starting = LogitechF310.DPAD_DOWN;
    static final int substationPickup = LogitechF310.DPAD_UP;

    static final double armXDead = RobotConfig.flightstickXDead;
    static final double armYDead = RobotConfig.flightstickYDead;

    public static Vector2 power;

    public static boolean autoRotating;

    //whether or not the robot is parked
    private static boolean parked;

    static boolean drivingToNode;
    public static Piece nodeTargetPiece;

    static boolean armMode;
    static boolean isPrimed;
    public static Piece gamePieceMode;

    public static void init() {
        power = new Vector2(0,0);
        driverStick = new Joystick(0);
        buttonBoard = new Joystick(1);
        flightStick = new Joystick(2);
        autoRotating = false;
        drivingToNode = false;
        gamePieceMode = Piece.CONE; 
        Manipulator.setConeMode(); 
    }

    public static void joystickInput() {
        runDriver();
        runOperator();
    }

    public static void runDriver() {

        double rotate = RMath.smoothJoystick1(driverStick.getRawAxis(rotateX)) * -0.3 + flightStick.getRawAxis(fineRotateX) * -0.05;

        // Rotation POV
        if (Math.abs(rotate) < 0.005) {
            rotate = 0;

            int POV = driverStick.getPOV();
            if (POV != -1) AutoAlign.runPOV(POV);
            if (autoRotating) rotate = SwervePID.updateOutputRot();

        } else { // If driver tries to rotate at all cancel autoRotate
            autoRotating = false;
        }

        // Funny button
        if (driverStick.getRawButtonPressed(driveToNode)) {
            drivingToNode = !drivingToNode;
            Vision.setLimelightLED(drivingToNode);
            if (drivingToNode) {
                Vector2 pos = SwervePosition.getPosition();
                double min = Double.MAX_VALUE;
                int minI = -1;
                for(int i = 0; i < 9; i++) {
                    double mag = Scoring.getScoringTarget(i).sub(pos).mag();
                    if(mag < min) {
                        min = mag;
                        minI = i;
                    }
                }
                nodeTargetPiece = ((minI - 2) % 3 == 0) ? Piece.CUBE : Piece.CONE; // use that stupid math done above to determine best scoring position

                // i know this vector compensates for something 
                // but i do not know what that is
                // and i can not figure that out.
                // it works so do not touch
                Vector2 magic = new Vector2(0.8, 3);

                SwervePID.setDestPt(Scoring.getScoringTarget(minI).add(magic));
                SwervePID.setDestRot(3 * Math.PI / 2);

                Telemetry.log(Severity.DEBUG, "" + nodeTargetPiece);
            }
        }

        // Zero the Pigeon
        if (driverStick.getRawButton(pigeonZero)) Pigeon.zero();

        // Auto Balancing in Teleoperated mode
        if (driverStick.getRawButtonPressed(balance)) {
            Auto.balance();
            Robot.teleauto = true;
        }

        // Boost
        double boostCoefficient = RobotConfig.joystickMoveScale;
        if (driverStick.getRawAxis(boost) > .5) boostCoefficient = 1;

        // Swerve fine control
        // Slows down the drivetrain
        if(driverStick.getRawAxis(LogitechF310.AXIS_LEFT_TRIGGER) > 0.5){
            boostCoefficient = 0.1;
            rotate *= 0.3;
        }

        // Drive vector
        // This is the input from left joystick
        Vector2 drive = new Vector2(driverStick.getRawAxis(moveX), -driverStick.getRawAxis(moveY));

        // Create a joystick deadzone so we don't move when the joystick isn't touched
        if (drive.mag() < 0.05) {
            drive = new Vector2(0, 0);
        } else {
            drive = RMath.smoothJoystick2(drive).mul(boostCoefficient);
            drivingToNode = false;
            Vision.setLimelightLED(drivingToNode);
        }

        SwerveInstruction instruction;

        if (!drivingToNode) {

            Vector2 diff = drive.sub(power);
            if (diff.mag() < RobotConfig.rampingCoefficent) {
                power = drive;
            } else {
                diff = diff.norm().mul(RobotConfig.rampingCoefficent);
                power = power.add(diff);
            }
            
            // We aren't using PID so the instruction
            // is just the driver input.
            instruction = new SwerveInstruction(rotate, power);
        } else {
            instruction = AutoAlign.funnyButton();
        }

        // Swerving and a steering! Zoom!
        SwerveManager.rotateAndDrive(instruction);
    }

    public static void runOperator() {
        double wristAdjust = flightStick.getRawAxis(LogitechF310.AXIS_LEFT_Y);
        if (Math.abs(wristAdjust) > 0.2) {
            Arm.destAngs[2] -= Math.PI * wristAdjust / 65.0;
        }

        if (flightStick.getPOV() == starting) {
            ArmStateController.controlState(ArmPosition.STARTING);
        } else if (flightStick.getRawButtonPressed(groundPickup)) {
            switch (gamePieceMode) {
                case CUBE: ArmStateController.controlState(ArmPosition.GROUND); break;
                case CONE: ArmStateController.controlState(ArmPosition.GROUND); break;
            }
            isPrimed = false;
        } else if (flightStick.getRawButtonPressed(groundScoring)) {
            switch (gamePieceMode) {
                case CUBE: ArmStateController.controlState(ArmPosition.CUBE_BOTTOM); break;
                case CONE: ArmStateController.controlState(ArmPosition.CONE_BOTTOM); break;
            }
            isPrimed = false;
        } else if (flightStick.getRawButtonPressed(midScoring)) {
            switch (gamePieceMode) {
                case CUBE: ArmStateController.controlState(ArmPosition.CUBE_MIDDLE); break;
                case CONE: ArmStateController.controlState(ArmPosition.CONE_MIDDLE); break;
            }
            isPrimed = false;
        }else if(flightStick.getRawButtonPressed(topScoring)) {
            switch (gamePieceMode) { 
                case CUBE: ArmStateController.controlState(ArmPosition.CUBE_TOP); break;
                case CONE: ArmStateController.controlState(ArmPosition.CONE_TOP); break;
            }
            isPrimed = false;
        } else if (flightStick.getRawButton(primed)) {
            switch (gamePieceMode) {
                case CUBE: ArmStateController.controlState(ArmPosition.PRIMED); break;
                case CONE: ArmStateController.controlState(ArmPosition.PRIMED); break;
            }
            isPrimed = true;
        } else if (flightStick.getRawButtonPressed(armDisable)) {
            Arm.setState(ArmControlMode.NEUTRAL, null, 0);
        } else if (flightStick.getPOV() == substationPickup) {
            switch (gamePieceMode) {
                case CUBE: ArmStateController.controlState(ArmPosition.SUBSTATION); break;
                case CONE: ArmStateController.controlState(ArmPosition.SUBSTATION); break;
            }
        }
        
        //when modeswitch is pressed if in cube mode switch to cone mode and vise versa
        if (flightStick.getRawButtonPressed(modeSwitch)) {
            switch (gamePieceMode) {
                case CUBE: gamePieceMode = Piece.CONE; Manipulator.setConeMode(); break;
                case CONE: gamePieceMode = Piece.CUBE; Manipulator.setCubeMode(); break;
            }
        }

        if(flightStick.getRawAxis(releasePiece) > 0.5) {
            switch (gamePieceMode) {
                case CUBE: 
                    if(ArmStateController.currentState==ArmPosition.SUBSTATION) {
                        Arm.destAngs[2] = ArmPosition.SUBSTATION.angles[2] + Math.toRadians(60);
                    }
                    else {
                        Manipulator.outtake(); 
                    }
                    break;
                case CONE: 
                    Manipulator.setOpen();
                    if(ArmStateController.currentState==ArmPosition.SUBSTATION) {
                        Arm.destAngs[2] = ArmPosition.SUBSTATION.angles[2];
                    }
                break;
            }
        } else if (flightStick.getRawAxis(grabPiece) > 0.5) {
            switch(gamePieceMode) {
                case CUBE: 
                    if(ArmStateController.currentState==ArmPosition.SUBSTATION) {
                        Arm.destAngs[2] = ArmPosition.SUBSTATION.angles[2] + Math.toRadians(60);
                    }
                    else {
                        Manipulator.intake();
                    }
                    break;
                case CONE: 
                    Manipulator.setConeMode(); 
                    if (ArmStateController.currentState==ArmPosition.SUBSTATION) {
                        Arm.destAngs[2] = ArmPosition.SUBSTATION.angles[2] + Math.toRadians(60);
                    }
                break;
            }
            // Manipulator.intake();
        } else {
            switch (gamePieceMode) {
                case CUBE: break;
                case CONE: Manipulator.coneModeAutomatic(); break;
            }
            Manipulator.neutral();
        }

        if (Arm.wristMotor.isRevLimitSwitchClosed() == 0) {
            if (ArmStateController.currentState==ArmPosition.SUBSTATION) {
                Arm.destAngs[2] = ArmPosition.SUBSTATION.angles[2] + Math.toRadians(60);
            }
        }

        // if(flightStick.getPOV()==LogitechF310.DPAD_RIGHT){
        //     Arm.recalibrateWrist();
        // }

        if (flightStick.getPOV() == LogitechF310.DPAD_LEFT) {
            switch (gamePieceMode) {
                case CUBE: ArmStateController.controlState(ArmPosition.CUBE_BOTTOM); break;
                case CONE: ArmStateController.controlState(ArmPosition.CONE_BOTTOM); break;
            }
        }
    }
}
