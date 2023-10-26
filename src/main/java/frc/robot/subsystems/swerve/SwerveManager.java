package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import frc.robot.RobotConfig;
import frc.robot.subsystems.Pigeon;
import frc.robot.utils.Vector2;

public class SwerveManager {

    public static SwerveMod[] m_swerveMods;
    public static boolean tipBlockEnabled;

    public static void init() {
        m_swerveMods = new SwerveMod[] {
            // Back right
            new SwerveMod(RobotConfig.steerID0, RobotConfig.driveID0, RobotConfig.swerveModX0, RobotConfig.swerveModY0, RobotConfig.cancoderOffset0, RobotConfig.falconOffset), 
            // Back left
            new SwerveMod(RobotConfig.steerID1, RobotConfig.driveID1, RobotConfig.swerveModX1,  RobotConfig.swerveModY1,  RobotConfig.cancoderOffset1, RobotConfig.falconOffset),
            // Front left
            new SwerveMod(RobotConfig.steerID2, RobotConfig.driveID2,  RobotConfig.swerveModX2,  RobotConfig.swerveModY2,  RobotConfig.cancoderOffset2, RobotConfig.falconOffset),
            // Front right
            new SwerveMod(RobotConfig.steerID3, RobotConfig.driveID3,  RobotConfig.swerveModX3, RobotConfig.swerveModY3,  RobotConfig.cancoderOffset3, RobotConfig.falconOffset),
        };
        tipBlockEnabled = true;
    }
    
    // Zero the encoder output of each of the steering motors
    public static void zeroSteeringEncoders() {
        for (SwerveMod mod : m_swerveMods) {
            mod.resetSteerSensor();
        }
    }

    public static void rotateAndDrive(SwerveInstruction si){
        rotateAndDrive(si.rotation, si.movement);
    }

    public static void rotateAndDrive(double rotSpeed, Vector2 move) {

        double heading = Pigeon.getRotationRad();
        
        // Array containing the unclamped movement vectors of each module
        Vector2[] vectors = new Vector2[m_swerveMods.length];

        // Multiply the movement vector by a rotation matrix to compensate for the pigeon's heading
        Vector2 relMove = move.rotate(-(heading - Math.PI / 2));

        // The greatest magnitude of any module's distance from the center of rotation
        double maxModPosMagnitude = 0;
        for (int i = 0; i < m_swerveMods.length; i++) {
            maxModPosMagnitude = Math.max(maxModPosMagnitude,
                    m_swerveMods[i].pos.mag());
        }

        // The greatest speed of the modules. If any one module's speed is
        // greater than 1.0, all the speeds are scaled down.
        double maxSpeed = 1.0;

        // Calculate unclamped movement vectors
        for (int i = 0; i < m_swerveMods.length; i++) {
            // The vector representing the direction the module should move to achieve the
            // desired rotation. Calculated by taking the derivative of the module's
            // position on the circle around the center of rotation, normalizing the
            // resulting vector according to maxModPosMagnitude (such that the magnitude of
            // the largest vector is 1), and scaling it by a factor of rotSpeed.

            Vector2 rotate = new Vector2(
                (-1 * m_swerveMods[i].pos.y / maxModPosMagnitude) * rotSpeed,
                (     m_swerveMods[i].pos.x / maxModPosMagnitude) * rotSpeed);

            // The final movement vector, calculated by summing movement and rotation
            // vectors

            Vector2 rotMove = relMove.add(rotate);

            vectors[i] = rotMove;
            maxSpeed = Math.max(maxSpeed, rotMove.mag());
        }

        for (int i = 0; i < m_swerveMods.length; i++) {
            // Convert the movement vectors to a directions and magnitudes, clamping the
            // magnitudes based on 'max'. 
            double direction = vectors[i].atan2();
            double power = vectors[i].mag() / maxSpeed;

            // Drive the swerve modules
            if(power != 0)
                m_swerveMods[i].rotateToRad(direction);
            m_swerveMods[i].drive(power);
        }
    }


    /**
     * Gets the raw encoder position of a specified SwerveMod's drive motor
     * @param id the ID of the SwerveModule to check
     * @return the raw encoder position, in ticks
     */
    public static double getEncoderPos(int id){
        return m_swerveMods[id].drive.getSelectedSensorPosition();
    }

    /**
     * Gets the velocity a given SwerveModule is driving at
     * @param id the ID of the SwerveModule to check
     * @return the drive velocity of the SwerveModule, in inches/second
     */
    public static double getDriveVelocity(int id) {
        return m_swerveMods[id].getDriveVelocity();
    }

    /**
     * Returns the angle a given SwerveModule's wheel is pointed toward
     * @param id the ID of the SwerveModule to check
     * @return the angle of the SwerveModule, in radians
     */
    public static double getSteerAngle(int id) {
        return m_swerveMods[id].getSteerAngle();
    }

    /**
     * Returns the overall drive velocity of the robot, based on the average of the velocities of the wheels. Not
     * adjusted for the rotation of the robot on the field.
     * @return the robot's drive velocity, in inches/second
     */
    public static Vector2 getRobotDriveVelocity() {
        Vector2 velSum = new Vector2();
        for (SwerveMod mod : m_swerveMods) {
            velSum = velSum.add(Vector2.fromPolar(mod.getSteerAngle(), mod.getDriveVelocity()));
        }

        return velSum.div(m_swerveMods.length);
    }

    /**
     * Returns the overall rotational velocity of the robot, based on the rotations and velocities of each of the
     * wheels. Primarily meant for applications where the Pigeon is unavailable, such as simulation.
     * @return the rotational velocity of the robot, in radians/second
     */
    public static double getRotationalVelocity() {
        // We only need to check the first swerve module
        Vector2 moduleVel = Vector2.fromPolar(getSteerAngle(0), getDriveVelocity(0));
        Vector2 rotVector = moduleVel.sub(getRobotDriveVelocity());

        // Get the one-dimensional rotation velocity in inches/sec, moving around the circle
        double rotVelInches = rotVector.rotate(-m_swerveMods[0].pos.atan2()).y;

        // Divide by the radius to get the rotation velocity in radians/sec
        double radius = m_swerveMods[0].pos.mag();
        return rotVelInches / radius;
    }

    public static void pointWheels(double radians) {
        for (SwerveMod mSwerveMod : m_swerveMods) mSwerveMod.rotateToRad(radians);
    }

    public static void calibrationTest() {
        for (SwerveMod mSwerveMod : m_swerveMods) {
            mSwerveMod.steer.set(TalonFXControlMode.MotionMagic, 0);
            mSwerveMod.drive.set(TalonFXControlMode.PercentOutput, 0.1);
        }
    }

    /**
     * Locks the robot in position by rotating all wheels towards the center of the robot
     */
    public static void lockWheels(){
        for(SwerveMod mSwerveMod : m_swerveMods){
            mSwerveMod.steer.setNeutralMode(NeutralMode.Brake);
            mSwerveMod.drive.setNeutralMode(NeutralMode.Brake);
            mSwerveMod.rotateToRad(mSwerveMod.pos.atan2());
            mSwerveMod.drive.neutralOutput();

        }
    }
}
