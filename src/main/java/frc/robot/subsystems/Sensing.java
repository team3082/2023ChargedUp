// package frc.robot.subsystems;

// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.Rev2mDistanceSensor.Port;
// import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
// import com.revrobotics.Rev2mDistanceSensor.Unit;

// /**
//  * A class for all items related to independent sensors (excluding encoders)
//  */
// public class Sensing {

//     // Distance sensor
//     public static Rev2mDistanceSensor distanceSensor;

//     public static void init() {
//         // Create distance sensor object
//         distanceSensor = new Rev2mDistanceSensor(Port.kMXP, Unit.kMillimeters, RangeProfile.kLongRange);
//         distanceSensor.setAutomaticMode(true);
//         // distanceSensor.
//     }

//     /**
//      * Gets the range of the distance sensor, returned in millimeters.
//      * I don't know what the isRangeValid() thing is, but we need it. The function will throw an exception if the range is invalid. 
//      * @param mode Mode of the sensor. True = millimeters, False = inches.
//      * @return the distance as recorded by the sensor
//      */
//     public static double getDistance(Unit mode) throws Exception {
//         double range = distanceSensor.getRange(mode); // THIS NEEDS TO BE ABOVE BECAUSE JOHN REV IS AN IDIOT
//         if (!distanceSensor.isRangeValid()) { // gets whether the LAST range reading was valid
//           // Telemetry.log(Telemetry.Severity.WARNING, "Distance sensor range is invalid: " + distanceSensor.GetRange());
//            throw new Exception("Distance sensor range is invalid.");
//         }
//         return range;
//     }
// }
