package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.AutoSelection;
import frc.robot.utils.Piece;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

import edu.wpi.first.wpilibj.RobotBase;

// Class for monitoring and relaying real-time robot data
public class Telemetry {

    static final double IN_TO_M = 1 / 39.37;

    // Status of the robot
    // True: Fully teleoperated.
    // False: Doing something autonomously? idk
    // When initialized, we should have full control.
    public static boolean rControl = true;

    // The robot's PDP. Ours is CTRE, so our ID is 0.
    public static PowerDistribution robotPD;

    // Colors for logging
    private static final String ANSI_RED = "\u001B[31m";
    private static final String ANSI_GREEN = "\u001B[32m";
    private static final String ANSI_YELLOW = "\u001B[33m";
    private static final String ANSI_RESET = "\u001B[0m";

    /**
     * Message severity options
     */
    public enum Severity {
        /**
         * Severity stating this message is purely informational.
         */
        INFO,
        /**
         * Severity stating this message contains general debugging information
         */
        DEBUG,
        /**
         * Severity stating this message is a warning.
         */
        WARNING, 
        /**
         * Severity stating this message conveys a critical/fatal error.
         */
        CRITICAL
    }

    // The Shuffleboard tabs that we use for monitoring data.
    // Each tab is isolated from one another.
    // This allows us to filter through only the ones we want to see,
    // and avoids any potential screwups.
    private static final ShuffleboardTab robotTab = Shuffleboard.getTab("SmartDashboard");
    private static final ShuffleboardTab pigeonTab = Shuffleboard.getTab("Pigeon");
    private static final ShuffleboardTab moveTab = Shuffleboard.getTab("Move PID");
    private static final ShuffleboardTab rotTab = Shuffleboard.getTab("Rot PID");
    private static final ShuffleboardTab shldrPID = Shuffleboard.getTab("Shoulder PID");
    private static final ShuffleboardTab elbowPID = Shuffleboard.getTab("Elbow PID");
    private static final ShuffleboardTab affTuning = Shuffleboard.getTab("AFF Values");
    private static final ShuffleboardTab pos = Shuffleboard.getTab("Positions");
    private static final ShuffleboardTab temps = Shuffleboard.getTab("Temperatures");
    private static final ShuffleboardTab power = Shuffleboard.getTab("Power Values");

    // NetworkTable entries
    // If we want granular control over our values via Glass (e.g, tuning PID),
    // We must use the NetworkTableEntry type, 
    // Since this enables live updates between Glass and our code. 

    // Field position
    private static final Field2d field = new Field2d();
    private static Vector2 prevSimPos = new Vector2();
    private static Rotation2d prevSimRot = new Rotation2d();

    // Arm display
    private static final Mechanism2d arm = new Mechanism2d(3, 2);
    private static final MechanismRoot2d root = arm.getRoot("shoulder", 1.0, Arm.shldrPos.y * IN_TO_M);

    // Target arm position
    private static final MechanismLigament2d tgtBicep =
            root.append(new MechanismLigament2d("bicep (target)", Arm.bicepLen * IN_TO_M, 0, 1, new Color8Bit(Color.kRed)));
    private static final MechanismLigament2d tgtForearm =
            tgtBicep.append(new MechanismLigament2d("forearm (target)", Arm.forearmLen * IN_TO_M, 0,  1, new Color8Bit(Color.kRed)));
    private static final MechanismLigament2d tgtManip =
            tgtForearm.append(new MechanismLigament2d("manipulator (target)", Arm.manipLen * IN_TO_M, 0,  1, new Color8Bit(Color.kRed)));

    // Actual arm position
    private static final MechanismLigament2d actBicep =
            root.append(new MechanismLigament2d("bicep (actual)", Arm.bicepLen * IN_TO_M, 0, 4, new Color8Bit(Color.kCornflowerBlue)));
    private static final MechanismLigament2d actForearm =
            actBicep.append(new MechanismLigament2d("forearm (actual)", Arm.forearmLen * IN_TO_M, 0, 4, new Color8Bit(Color.kCornflowerBlue)));
    private static final MechanismLigament2d actManip =
            actForearm.append(new MechanismLigament2d("manipulator (actual)", Arm.manipLen * IN_TO_M, 0, 5, new Color8Bit(Color.kYellow)));

    // Move PID
    private static final GenericEntry moveP = moveTab.add("Move P", SwervePID.moveP).getEntry();
    private static final GenericEntry moveI = moveTab.add("Move I", SwervePID.moveI).getEntry();
    private static final GenericEntry moveD = moveTab.add("Move D", SwervePID.moveD).getEntry();
    private static final GenericEntry moveDeadband = moveTab.add("Move Deadband", SwervePID.moveDead).getEntry();

    // Rot PID
    private static final GenericEntry rotP = rotTab.add("Rot P", SwervePID.rotP).getEntry();
    private static final GenericEntry rotI = rotTab.add("Rot I", SwervePID.rotI).getEntry();
    private static final GenericEntry rotD = rotTab.add("Rot D", SwervePID.rotD).getEntry();
    private static final GenericEntry rotDeadBand = rotTab.add("Rot Deadband", SwervePID.rotDead).getEntry();

    // Shoulder PID
    private static final GenericEntry shP = shldrPID.add("Shoulder P", Arm.sP).getEntry();
    private static final GenericEntry shI = shldrPID.add("Shoulder I", Arm.sI).getEntry();
    private static final GenericEntry shD = shldrPID.add("Shoulder D", Arm.sD).getEntry();

    // Elbow PID
    private static final GenericEntry eP = elbowPID.add("Elbow P", Arm.eP).getEntry();
    private static final GenericEntry eI = elbowPID.add("Elbow I", Arm.eI).getEntry();
    private static final GenericEntry eD = elbowPID.add("Elbow D", Arm.eD).getEntry();

    // AFF stuff
    private static final GenericEntry wristAFF = affTuning.add("Wrist AFF", Arm.wristAFFconst).getEntry();
    private static final GenericEntry elbowAFF = affTuning.add("Elbow AFF", Arm.elbowAFFconst).getEntry();
    private static final GenericEntry shldrAFF = affTuning.add("Shoulder AFF", Arm.shldrAFFconst).getEntry();
    
    // Auto stuff
    private static final GenericEntry robotBool = robotTab.add("Operator Control", rControl).getEntry();

    // Arm positions
    private static final GenericEntry shldrPos = pos.add("Shoulder Position", Arm.getShldrTheta()).getEntry();
    private static final GenericEntry elbowPos = pos.add("Elbow Position", Arm.getElbowTheta()).getEntry();
    private static final GenericEntry wristPos = pos.add("Wrist Position", Arm.getWristTheta()).getEntry();

    // SwervePosition
    private static final GenericEntry swervePos = pos.add("Swerve Position", SwervePosition.getPosition().toString()).getEntry();

    // Temperatures
    private static final GenericEntry wristTemp = temps.add("Wrist Temperatures", Arm.wristTemp).getEntry();
    private static final GenericEntry intakeTemp = temps.add("Intake Temperature", Manipulator.intakeTemp).getEntry();

    // Power monitoring
    private static final GenericEntry intakePower = power.add("Intake Current", Manipulator.intakeCurrent).getEntry();

    // Public object attached to the wpilib data logger
    public static DataLog FRClogger;

    // roboRIO logging directory representing critical/fatal logs
    private static StringLogEntry criticalLogs;

    // roboRIO logging directory representing moderately important logs
    private static StringLogEntry warningLogs;

    // roboRIO logging directory representing logs for general debugging info
    private static StringLogEntry debugLogs;

    // roboRIO logging directory representing informational logs
    private static StringLogEntry infoLogs;

    public static void init() {

        // Input other misc values into Shuffleboard.
        pigeonTab.add("Pigeon", Pigeon.pigeon);
        robotTab.add("Field View", field);
        robotTab.add("Arm View", arm);
        robotTab.add("Auto Selector", AutoSelection.autoChooser);
        robotTab.add("Mode Selector", AutoSelection.pieceMode);
        robotPD = new PowerDistribution(0, ModuleType.kCTRE);
        power.add("PDP", robotPD);

        // Start WPILib data logging thingy
        DataLogManager.start();
        FRClogger = DataLogManager.getLog();

        // Create logging directories on the roboRIO
        criticalLogs = new StringLogEntry(FRClogger, "/logs/critical");
        warningLogs = new StringLogEntry(FRClogger, "/logs/warning");
        debugLogs = new StringLogEntry(FRClogger, "/logs/debug");
        infoLogs = new StringLogEntry(FRClogger, "/logs/info");
    }

    /**
     * Log a message to the console. Same as a print statement, however this has color, and it is prefixed with a timestamp.
     * @param severity The severity of the message as a string. This changes the color of the message.
     * @param subsystem The subsystem of which this message was derived from.
     * @param message The message to send to the console.
     */
    public static void log(Severity severity, String subsystem, String message) {
        StringLogEntry targetLog;
        String color;

        switch (severity) {
            case INFO:
                color = ANSI_RESET;
                targetLog = infoLogs;
            break;
            case DEBUG:
                color = ANSI_GREEN;
                targetLog = debugLogs;
            break;
            case WARNING:
                color = ANSI_YELLOW;
                targetLog = warningLogs;
            break;
            case CRITICAL:
                color = ANSI_RED;
                targetLog = criticalLogs;
            break;
            default:
                return;
        }
        String fullMessage = "[" + RTime.createTimestamp() + "]" + " [" + severity + "] " + "(" + subsystem + "):" + " " + message;
        System.out.println(color + fullMessage + ANSI_RESET);
        targetLog.setMetadata(fullMessage);
    }

    /**
     * Log a message to the console. Same as a print statement, however this has color, and it is prefixed with a timestamp.
     * The name of the class that called this function is automatically detected and printed as well.
     * @param severity The severity of the message as a string. This changes the color of the message.
     * @param message The message to send to the console.
     */
    public static void log(Severity severity, String message) {
        String caller = Thread.currentThread().getStackTrace()[2].getClassName();
        log(severity, caller, message);
    }

    /**
     * Updates and reads from telemetry. Should be called each frame
     * @param compMode whether we should enable a lightweight version of telemetry for competition. 
     * This is just so we don't hog network bandwidth,
     * and it still gives us decently important information.
     */
    public static void update(boolean compMode) {

        // -1 if we're on the red alliance, 1 if we're on the blue alliance
        int allianceMultiplier = (DriverStation.getAlliance() == DriverStation.Alliance.Red) ? -1 : 1;

        // Raw values for the various encoder positions.
        // Not that I don't trust Nick's cracked code,
        // but seeing values like this is better than print statements...
        shldrPos.setDouble(Arm.getShldrTheta());
        elbowPos.setDouble(Arm.getElbowTheta());
        wristPos.setDouble(Arm.getWristTheta());

        swervePos.setString(SwervePosition.getPosition().toString());

        // Power stuff
        intakePower.setDouble(Manipulator.intakeCurrent);

        // Update arm position and colors
        if (Manipulator.getMode() == Piece.CUBE) {
            actManip.setColor(new Color8Bit(Color.kPurple));
        } else {
            actManip.setColor(new Color8Bit(Color.kYellow));
        }

        tgtBicep.setAngle(Arm.getShldrDest() * 180 / Math.PI);
        tgtForearm.setAngle(Arm.getElbowDest() * 180 / Math.PI);
        tgtManip.setAngle(Arm.getWristDest() * 180 / Math.PI);

        actBicep.setAngle(Arm.getShldrTheta() * 180 / Math.PI);
        actForearm.setAngle(Arm.getElbowTheta() * 180 / Math.PI);
        actManip.setAngle(Arm.getWristTheta() * 180 / Math.PI);

        // Temperature stuff
        wristTemp.setDouble(Arm.wristTemp);
        intakeTemp.setDouble(Manipulator.intakeTemp);

        try {
            //distance.setDouble(Sensing.getDistance(Unit.kInches));
        } catch (Exception e) {
            //System.out.println(Sensing.distanceSensor.getRange(Unit.kInches));
        }

        // FIX
        robotBool.getBoolean(false);

        if (RobotBase.isSimulation()) {
            // Allow the user to drag the robot around if we're in simulation mode
            Vector2 modifiedSimPos = new Vector2(field.getRobotPose().getX(), field.getRobotPose().getY());
            if (prevSimPos.sub(modifiedSimPos).mag() > 0.001) {
                Vector2 modifiedSwervePos = modifiedSimPos
                        .div(IN_TO_M)
                        .sub(new Vector2(325.62, 157.75));
                SwervePosition.setPosition(new Vector2(-modifiedSwervePos.y, modifiedSwervePos.x * allianceMultiplier));
            }

            Rotation2d modifiedSimRot = field.getRobotPose().getRotation();
            if (Math.abs(prevSimRot.minus(modifiedSimRot).getRadians()) > 0.001)
                Pigeon.setSimulatedRot(modifiedSimRot.getRadians() + Math.PI / 2 * allianceMultiplier);
        }

        // Update field position and trajectory
        Vector2 fieldPosMeters = new Vector2(SwervePosition.getPosition().y * allianceMultiplier, -SwervePosition.getPosition().x)
                .add(new Vector2(325.62, 157.75))
                .mul(IN_TO_M);
        Rotation2d rotation = Rotation2d.fromRadians(Pigeon.getRotationRad() - Math.PI / 2 * allianceMultiplier);
        field.setRobotPose(fieldPosMeters.x, fieldPosMeters.y, rotation);

        // Store the field position for the next frame to check if it has been manually changed
        prevSimPos = fieldPosMeters;
        prevSimRot = rotation;

        if (compMode)
            return;

        // CODE BELOW HERE WILL NOT RUN IN COMP MODE ==================================================================================

        if (SwervePID.xPID.isActive() || SwervePID.yPID.isActive()) {
            // Build the robot's trajectory based on its current and target positions
            Vector2 targetPos = new Vector2(SwervePID.xPID.getDest(), SwervePID.yPID.getDest());
            Vector2 targetPosMeters = new Vector2(targetPos.y * allianceMultiplier, -targetPos.x)
                .add(new Vector2(325.62, 157.75))
                .mul(IN_TO_M);
            Rotation2d rot = Rotation2d.fromRadians(Math.atan2(targetPosMeters.y - fieldPosMeters.y, targetPosMeters.x - fieldPosMeters.x));
            Trajectory traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(fieldPosMeters.x, fieldPosMeters.y, rot),
                List.of(new Translation2d((fieldPosMeters.x + targetPosMeters.x + 0.001) / 2, (fieldPosMeters.y + targetPosMeters.y + 0.001) / 2)), // crashes without the 0.001 fsr
                new Pose2d(targetPosMeters.x, targetPosMeters.y, rot),
                new TrajectoryConfig(1.0, 1.0)
            );
            field.getObject("Trajectory").setTrajectory(traj);
        } else {
            // If we're not supposed to move, move the trajectory really far away so you can't see it
            Rotation2d rot = new Rotation2d();
            Trajectory traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(100.0, 100.0, rot),
                List.of(new Translation2d(100.0, 101.0)),
                new Pose2d(102.0, 102.0, rot),
                new TrajectoryConfig(1.0, 1.0)
            );
            field.getObject("Trajectory").setTrajectory(traj);
        }


        // Move PID: X
        SwervePID.xPID.kP = moveP.getDouble(0);
        SwervePID.xPID.kI = moveI.getDouble(0);
        SwervePID.xPID.kD = moveD.getDouble(0);
        SwervePID.xPID.deadband = moveDeadband.getDouble(0);

        // Move PID: Y
        SwervePID.yPID.kP = moveP.getDouble(0);
        SwervePID.yPID.kI = moveI.getDouble(0);
        SwervePID.yPID.kD = moveD.getDouble(0);
        SwervePID.yPID.deadband = moveDeadband.getDouble(0);

        // Rot PID
        SwervePID.rotPID.kP = rotP.getDouble(0);
        SwervePID.rotPID.kI = rotI.getDouble(0);
        SwervePID.rotPID.kD = rotD.getDouble(0);
        SwervePID.rotPID.deadband = rotDeadBand.getDouble(0);

        // Shoulder PID
        Arm.sP = shP.getDouble(0);
        Arm.sI = shI.getDouble(0);
        Arm.sD = shD.getDouble(0);

        // Elbow PID
        Arm.eP = eP.getDouble(0);
        Arm.eI = eI.getDouble(0);
        Arm.eD = eD.getDouble(0);

        // AFF stuff
        Arm.elbowAFFconst = elbowAFF.getDouble(0);
        Arm.wristAFFconst = wristAFF.getDouble(0);
        Arm.shldrAFFconst = shldrAFF.getDouble(0);
    }
}
