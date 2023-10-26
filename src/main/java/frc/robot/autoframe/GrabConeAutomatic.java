// package frc.robot.autoframe;

// import com.revrobotics.Rev2mDistanceSensor.Unit;

// import frc.robot.RobotConfig;
// import frc.robot.subsystems.Manipulator;
// import frc.robot.subsystems.Sensing;
// import frc.robot.utils.RTime;


// public class GrabConeAutomatic extends AutoFrame {
//     private double timeout;
//     private double stopTime;

//     /**
//      * Instructs the manipulator to intake for a set duration in seconds. 
//      * @param duration the number of seconds the manipulator should stay active.
//      */
//     public GrabConeAutomatic(double timeout){
//         this.timeout = timeout;
//     }

//     public GrabConeAutomatic(){
//         this(Double.MAX_VALUE);
//     }

//     @Override
//     public void start(){
//         stopTime = RTime.now() + timeout;
//         Manipulator.setCubeMode();
//     }

//     @Override
//     public void update(){
//         boolean inManip = false;
//         try {
//             inManip = Sensing.getDistance(Unit.kInches) < RobotConfig.autoGrabDist;
//         } catch (Exception e) { }

//         if (inManip || RTime.now() > this.stopTime){
//             Manipulator.setConeMode();
//             this.done = true;
//         }
//     }

//     @Override
//     public void finish() {
//         Manipulator.neutral();
//     }
// }
