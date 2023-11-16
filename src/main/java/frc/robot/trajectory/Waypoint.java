package frc.robot.trajectory;

import frc.robot.subsystems.arm.ArmStateController;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.utils.Vector2;

public class Waypoint {
    // Should represent a Vector2 for the point
    // and an action that should occur at the point. 
    // Somehow need to integrate that.

    public enum actions {
        PRIMED("PRIMED"),
        MID("MID"),
        HIGH("HIGH"),
        START("START"),
        BOTTOM("BOTTOM"),
        NONE("NONE"),
        GROUND("GROUND");

        private String action;

        actions(String a) {
            this.action = a;
        }
    }

    public static actions selectedAction;
    public Vector2 point;
    public double rotation;

    public Waypoint(Vector2 p, double rot, actions desiredAction) {
        point = p;
        selectedAction = desiredAction;
        rotation = rot;
    }

    public void selectAction() {
        switch (selectedAction) {
            case PRIMED:
                ArmStateController.controlState(ArmPosition.PRIMED);
                Telemetry.log(Telemetry.Severity.INFO, "WAYPOINT", "GOING TO PRIMED");
            break;
            case MID:
                ArmStateController.controlState(ArmPosition.CONE_MIDDLE);
                Telemetry.log(Telemetry.Severity.INFO, "WAYPOINT", "GOING TO MID");
            break;
            case HIGH:
                ArmStateController.controlState(ArmPosition.CONE_TOP);
                Telemetry.log(Telemetry.Severity.INFO, "WAYPOINT", "GOING TO HIGH");
            break;
            case START:
                ArmStateController.controlState(ArmPosition.STARTING);
                Telemetry.log(Telemetry.Severity.INFO, "WAYPOINT", "GOING TO STARTING");
            break;
            case BOTTOM:
                ArmStateController.controlState(ArmPosition.CONE_BOTTOM);
                Telemetry.log(Telemetry.Severity.INFO, "WAYPOINT", "GOING TO BOTTOM");
            break;
            case NONE:
                Telemetry.log(Telemetry.Severity.INFO, "WAYPOINT", "NOTHING");
            break;
            case GROUND:
                ArmStateController.controlState(ArmPosition.GROUND);
            break;
        }
    }
}
