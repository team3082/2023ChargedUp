package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConfig;
import frc.robot.utils.Vector2;

public class NTVision {
    
    // Position in inches straight from the game manual. See https://www.desmos.com/calculator/ctcioi3gyj
    private static final Vector2[] aprilTags = {
        new Vector2(RobotConfig.firstX, RobotConfig.gridY),
        new Vector2(RobotConfig.secondX, RobotConfig.gridY),
        new Vector2(RobotConfig.thirdX, RobotConfig.gridY),
        new Vector2(RobotConfig.loadingX, RobotConfig.loadingY)
    };

    static int camNum = 1;

    // Vectors from center of robot to camera (in inces). +y is forward on robot & +x is right
    private static Vector2[] cameraOffsets = new Vector2[camNum];
    //PI/2 is straight forward
    private static double[] cameraRots = new double[camNum];

    public static NetworkTableEntry targetDetected;
    public static NetworkTableEntry targetXEntry;
    public static NetworkTableEntry targetYEntry;
    public static NetworkTableEntry targetZEntry;
    public static NetworkTableEntry tagIDEntry;

    public static double targetX;
    public static double targetY;
    public static double targetZ;
    public static double tagID;

    public static boolean visionActive;

    public static void init() {

        // Find our NetworkTable
        NetworkTableInstance NT = NetworkTableInstance.getDefault();
        NetworkTable table = NT.getTable("vision");
        
        // Fetch values from the Vision NetworkTable and import initial values
        targetDetected = table.getEntry("targetDetected");
        targetXEntry = table.getEntry("targetX");
        targetYEntry = table.getEntry("targetY");
        targetZEntry = table.getEntry("targetZ");
        tagIDEntry = table.getEntry("tagID");

        // Set our offsets for the cameras.
        cameraOffsets[0] = new Vector2(-11, 9);
        cameraRots[0] = Math.PI / 2;
    }

    /**
     * Updates our entries with new values from the NetworkTable.
     * This should be in robotPeriodic, as vision will always be "on",
     * but we can determine if vision is actively being used with the visionActive boolean.
     */
    public static void update() {

        targetX = targetXEntry.getDouble(0);
        targetY = targetYEntry.getDouble(0);
        targetZ = targetZEntry.getDouble(0);
        tagID = tagIDEntry.getDouble(0);
    }

    public static double getTagID() {
        return tagID;
    }

    public static double getX() {
        return targetX;
    }

    public static double getY() {
        return targetY;
    }

    /**
     * Uses vision to get the position of the robot on the field, in inches, 
     * with (0, 0) being the center of the field. Throws an exception if
     * there are no usable targets or if values are likely unreliable.
     * @return The position of the robot on the field.
     * @throws Exception
     */
    public static Vector2 getRobotPos() throws Exception {

        // The error of the camera when the robot is pressed up against the grid
        final double INCH_OFFSET_STARTING = 1.6;
        // The distance of the camera from the edge of the bumper 
        final double CAM_DIST_FROM_BUMPER = (RobotConfig.frameLength / 2.0) - cameraOffsets[0].y + 3.0;
        // The distance of the camera from the april tag when the robot is pressed up against the grid
        final double CAM_DIST_FROM_TAG_STARTING = CAM_DIST_FROM_BUMPER + 14.5;
        // The distance offset multiplier (offset accumulation per inch from the grid)
        final double DIST_OFFSET_MULTIPLIER = INCH_OFFSET_STARTING / CAM_DIST_FROM_TAG_STARTING;

        // If the robot's rotation speed is greater than 0.2, just return null because calculated values are likely unreliable
        if (Pigeon.getDeltaRotRad() > 0.2)
            throw new Exception("Robot rotating too fast! getRobotPos() ignored.");
            
        Vector2 posSum = new Vector2();
        int nTargets = 0;

        for(int i = 0; i < camNum; i++) {
            
            double id = getTagID();
            if (id > 8 || id < 1) 
                continue;

            // offset is the vector from center of the robot to the apriltag
            // offset.x is amt right & offset.y is amt forward
            Vector2 offset = new Vector2(-getY(), getX());
            offset = offset.mul(39.3701); //Convert to inches
            offset.y *= Math.cos(Math.toRadians(17.5));
            offset.x += DIST_OFFSET_MULTIPLIER * offset.y; // Point vector axes forward relative to robot. (Used if we have cameras at an angle)
            offset = offset.add(cameraOffsets[i]); //Make vector from center of robot, not from camera.
            offset = toFieldSpace(offset, Pigeon.getRotationRad(), id); // Make vector in field space instead of robot space

            Telemetry.log(Telemetry.Severity.INFO, "VISION", offset.toString());

            // Adding so we can average
            posSum = posSum.add(offset);
            nTargets++;
        }

        if (nTargets > 0) { // Only update if we actually saw any targets
            return posSum.div(nTargets);
        }

        throw new Exception("No targets found! getRobotPos() ignored.");
    }

    /**
     * Convert from a vector from center of robot to tag that is relative to robot rotation to
     * a vector from center of field to the robot relative to field rotation
     * @param offset the vector from the center of the robot to the tag, relative to the robot rotation
     * @param pigeonAngle the yaw of the pigeon in radians
     * @param tagID the ID of the AprilTag detected
     * @return a vector from the center of the field to the robot, relative to the field rotation
     */
    private static Vector2 toFieldSpace(Vector2 offset, double pigeonAngle, double tagID) {
        // tagRelOffset.x *= -1; // Source: trust me bro I drew it -Parker H. We just need to invert here (after we rotate to be in line with field)
        //tagRelOffset.x is amt right from tag we are & tagRelOffset.y is amt far in front of it from drivers' POV (If this is an enemy tag this should be negative)
        offset = offset.mul(-1);
        Vector2 tagRelOffset = offset.rotate(pigeonAngle - Math.PI / 2);

        // We want to flip the x of the offset from the tag, but not the position of the tag itself.
        if (DriverStation.getAlliance() == Alliance.Red)
                tagRelOffset.x *= -1;
        
        Vector2 absolutePos = getTagPos(tagID).add(tagRelOffset);

        return absolutePos;
    }

    public static boolean isTagFriendly(double tagID){
        switch (DriverStation.getAlliance()){
            case Red:
                return tagID < 5;
            case Blue:
                return tagID > 4;
            default:
                Telemetry.log(Telemetry.Severity.CRITICAL, "SOMETHING IS BAD WITH DRIVERSTATION ALLIANCE");
                return false;
        }
    }

    public static Vector2 getTagPos(double tagID) {
        double index = tagID < 5 ? tagID - 1 : 8 - tagID;
        
        // LEAVE IT LIKE THIS SO WE DON'T FLIP APRIL TAG POSITIONS
        Vector2 v = new Vector2(aprilTags[(int) index].x, aprilTags[(int) index].y);
        
        if(!isTagFriendly(tagID))
            v.y *= -1; //If it's enemy make tag y positive
        return v;
    }
}
