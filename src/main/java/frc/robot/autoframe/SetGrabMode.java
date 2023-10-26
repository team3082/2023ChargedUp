package frc.robot.autoframe;

import frc.robot.subsystems.Manipulator;
import frc.robot.utils.Piece;

public class SetGrabMode extends AutoFrame {    
    private Piece grabMode;
    
    /**
     * Adjusts the manipulator to prepare it to pick up a given Piece.
     * @param grabMode the Piece type to adjust for.
     */
    public SetGrabMode(Piece grabMode){
        this.grabMode = grabMode;
    }

    @Override
    public void start(){
        switch(grabMode){
            case CONE:
                Manipulator.setConeMode();
                break;
            case CUBE:
                Manipulator.setCubeMode();
        }
        this.done = true;
    }
}
