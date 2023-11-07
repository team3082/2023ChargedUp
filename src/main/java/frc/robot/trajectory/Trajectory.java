package frc.robot.trajectory;

import frc.robot.Robot;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.swerve.SwerveInstruction;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.Vector2;

public class Trajectory {

    public enum trajectoryProgress {
        AUTOSTART,
        NOTHING,
        MOVE_TO_START,
        AT_START,
        MOVING_WAYPOINT1,
        AT_WAYPOINT1,
        MOVING_WAYPOINT2,
        AT_WAYPOINT2,
        MOVING_END,
        AT_END
    }
    
    public Vector2 startPoint, endPoint;
    public Waypoint waypointOne, waypointTwo;
    public double speedMultiplier;
    public trajectoryProgress progress;

    public Trajectory(Vector2 s, Vector2 e, Waypoint w1, Waypoint w2, double speed) {
        startPoint = s;
        endPoint = e;
        waypointOne = w1;
        waypointTwo = w2;
        speedMultiplier = speed;
        progress = trajectoryProgress.MOVE_TO_START;
    }

    public void setSpeed(double newSpeed) {
        this.speedMultiplier = newSpeed;
    }

    public SwerveInstruction updateDest() {
        switch (progress) {
            case AUTOSTART:
                if (Robot.auto == true) progress = trajectoryProgress.MOVE_TO_START;
                Telemetry.log(Telemetry.Severity.INFO, "TRAJECTORY", "NOTHING");
            break;
            case MOVE_TO_START:
                SwervePID.setDestPt(startPoint);
                if(SwervePosition.getPosition().sub(SwervePID.getDest()).mag() < 5 && SwervePID.rotPID.getError() < Math.toRadians(2)){
                    progress = trajectoryProgress.AT_START;
                } else {
                    return SwervePID.updateAll();
                }
            break;
            case AT_START:
                Telemetry.log(Telemetry.Severity.INFO, "TRAJECTORY", "At start.");
                progress = trajectoryProgress.MOVING_WAYPOINT1;
            break;
            case MOVING_WAYPOINT1:
                SwervePID.setDestPt(waypointOne.point);
                //SwervePID.setDestRot(waypointOne.rotation); // rotPID is screwed up
                if(SwervePosition.getPosition().sub(SwervePID.getDest()).mag() < 5 && SwervePID.rotPID.getError() < Math.toRadians(2)){
                    progress = trajectoryProgress.AT_WAYPOINT1;
                } else {
                    return SwervePID.updateAll();
                }
            break;
            case AT_WAYPOINT1:
                Telemetry.log(Telemetry.Severity.INFO, "TRAJECTORY", "At Waypoint 1.");
                progress = trajectoryProgress.MOVING_WAYPOINT2;
            break;
            case MOVING_WAYPOINT2:
                SwervePID.setDestPt(waypointTwo.point);
                SwervePID.setDestRot(waypointOne.rotation); // rotPID is screwed up
                if(SwervePosition.getPosition().sub(SwervePID.getDest()).mag() < 5 && SwervePID.rotPID.getError() < Math.toRadians(2)){
                    progress = trajectoryProgress.AT_WAYPOINT2;
                } else {
                    return SwervePID.updateAll();
                }
            case AT_WAYPOINT2:
                Telemetry.log(Telemetry.Severity.INFO, "TRAJECTORY", "At Waypoint 2.");
                progress = trajectoryProgress.MOVING_END;
            break;
            case MOVING_END:
                SwervePID.setDestPt(endPoint);
                if(SwervePosition.getPosition().sub(SwervePID.getDest()).mag() < 5 && SwervePID.rotPID.getError() < Math.toRadians(2)){
                    waypointTwo.selectAction();
                    progress = trajectoryProgress.AT_END;
                } else {
                    return SwervePID.updateAll();
                }
            break;
            case AT_END:
                Telemetry.log(Telemetry.Severity.INFO, "TRAJECTORY", "At Final Point.");
                progress = trajectoryProgress.NOTHING;
            break;
            case NOTHING:
                
            break;
        }
        return new SwerveInstruction(); // Not commanding movement from Swerve Drive
    }
}
