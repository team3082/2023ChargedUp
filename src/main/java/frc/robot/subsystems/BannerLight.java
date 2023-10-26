package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Auto;
import frc.robot.Robot;
import frc.robot.autoframe.Balance;
import frc.robot.autoframe.Score;
import frc.robot.utils.Piece;

/*
list of stuff to do :))
cone mode - solid yellow
cube mode - solid purple
balancing - 50/50 rotate blue and red
auto - countdown in last five seconds
*/

public class BannerLight {
    public static DigitalOutput brown;
    public static DigitalOutput grey;
    public static DigitalOutput black;
    public static DigitalOutput white;

    public static void init(){
        grey = new DigitalOutput(0);
        white = new DigitalOutput(1);
        black = new DigitalOutput(2);
        brown = new DigitalOutput(3);
        
        grey.set(false);
        white.set(false);
        black.set(true);
        brown.set(false);
        // switch(Robot.getAlliance()){
        //     case Red:
        //     brown.set(false);
        //     break;
        //     case Blue:
        //     brown.set(true);
        //     break;
        //     case Invalid:
        //     brown.set(false);
        //     break;
        // }
    }

    public static void updateTeleop(){
        if(Robot.teleauto){
            setBalancing();
        } else if (OI.drivingToNode){
            setScoring();
        } else {
            setMode();
        }
    }
    public static void updateAuto(){
        if(Auto.activeFrames.contains(Balance.class)){
            setBalancing();
        } else if (Auto.activeFrames.contains(Score.class)){
            setScoring();
        } else {
            if(Manipulator.getMode()==Piece.CUBE)
                setCubeMode();
            else
                setConeMode();
        }
    }

    public static void setTagInView(boolean tagInView, Piece pieceHeld) {
        if (!tagInView) {
            black.set(true);
            white.set(false);
            grey.set(true);
            brown.set(false);    
        } else if (pieceHeld == Piece.CONE) {
            black.set(true);
            white.set(true);
            grey.set(false);
            brown.set(true); 
        } else {
            black.set(true);
            white.set(false);
            grey.set(true);
            brown.set(true); 
        }
    }

    public static void setCubeMode(){
        // grey
        grey.set(true);
        white.set(false);
        black.set(false);
        brown.set(false);
    }
    
    public static void setConeMode(){
        // white
        grey.set(false);
        white.set(true);
        black.set(false);
        brown.set(false);
    }

    public static void setIdle(){
        // black
        grey.set(false);
        white.set(false);
        black.set(true);
        brown.set(false);
    }

    public static void setBalancing(){
        // white & grey
        grey.set(true);
        white.set(true);
        black.set(false);
        brown.set(false);
    }

    public static void setScoring(){
        // grey & white
        grey.set(true);
        white.set(true);
        black.set(true);
        brown.set(false);
    }


    public static void setMode(){
        if(OI.gamePieceMode == Piece.CONE) setConeMode();
        else setCubeMode();
    }
}
