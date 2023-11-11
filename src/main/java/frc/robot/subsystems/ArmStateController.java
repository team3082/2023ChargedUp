package frc.robot.subsystems;

import frc.robot.subsystems.Arm.ArmPosition;

public class ArmStateController {

    public enum Type {
        NORMAL,
        PROFILED
    }

    public static Arm.ArmPosition currentState = ArmPosition.PRIMED;

    private static void startingController() {
        switch(currentState) {
            case PRIMED:
                Arm.setState(ArmPosition.STARTING);
                break;
            
            default:
                Arm.setStateIntermediate(ArmPosition.PRIMED, ArmPosition.STARTING);
                break;
        }
    }

    private static void coneTopController() {
        switch(currentState) {
            case STARTING:
                Arm.setState(ArmPosition.PRIMED);
                break;

            case GROUND:
                Arm.setState(ArmPosition.PRIMED);
                break;
            
            default:
                Arm.setStateIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.CONE_TOP);
                break;
        }   
    }

    private static void coneMidController() {
        switch(currentState) {
            case STARTING:
                Arm.setState(ArmPosition.PRIMED);
                break;

            case GROUND:
                Arm.setState(ArmPosition.PRIMED);
                break;
            
            default:
                Arm.setStateIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.CONE_MIDDLE);
                break;
        }   
    }

    private static void cubeTopController() {
        switch(currentState) {
            case STARTING:
                Arm.setState(ArmPosition.PRIMED);
                break;

            case GROUND:
                Arm.setState(ArmPosition.PRIMED);
                break;
            
            default:
                Arm.setStateIntermediate(ArmPosition.SCORE_INTERMEDIATE, ArmPosition.CUBE_TOP);
                break;
        }   
    }

    private static void defaultController(ArmPosition nextPosition) {
        switch(currentState) {
            case STARTING:
                Arm.setState(ArmPosition.PRIMED);
                break;

            case GROUND:
                Arm.setState(ArmPosition.PRIMED);
                break;

            default: 
                Arm.setState(nextPosition);
                break;
        }
    }

    public static void controlState(ArmPosition nextPosition) {
        switch(nextPosition) {

            case STARTING:
                startingController();
                break;

            case GROUND:
                Arm.setStateProfile(nextPosition);;
                break;

            case CONE_TOP:
                coneTopController(); 
                break;

            case CONE_MIDDLE:
                coneMidController();
                break;

            case CUBE_TOP: 
                cubeTopController();
                break;

            default:
                defaultController(nextPosition);
                break;
        }
    }
}
