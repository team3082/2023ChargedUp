package frc.robot.subsystems;

import frc.robot.subsystems.Telemetry.Severity;
import frc.robot.subsystems.swerve.SwerveInstruction;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.utils.Piece;
import frc.robot.utils.Vector2;

public class AutoAlign {
    
    /**
     * Instructs Swerve to turn the robot to a position
     * relative to the POV / D-Pad on the Xbox Controller
     * @param pov the value returned from the POV / D-Pad
     */
    public static void runPOV(double pov) {
        OI.autoRotating = true;
        SwervePID.setDestRot(Math.PI/2.0 - Math.toRadians(pov-180));
    }

    /**
     * Aligns to a scoring column.
     * Best used when wanting to score on a cone pole.
     * @return A SwerveInstruction frame containing the updated values
     * from SwervePID.
     */
    public static SwerveInstruction funnyButton() {
        double angleDeg = Vision.limelight.getXOffset();
        Telemetry.log(Severity.DEBUG, "" + OI.nodeTargetPiece);
        double velX = OI.nodeTargetPiece == Piece.CUBE ? SwervePID.updateOutputX() : angleDeg * -0.015; // i think this is so we dont hit the grid

        Vector2 movement = new Vector2(velX, SwervePID.updateOutputY());
        double rotation = SwervePID.updateOutputRot();

        return new SwerveInstruction(rotation, movement);
    }
}
