package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotConfig;
import frc.robot.subsystems.arm.ArmStateController;
import frc.robot.subsystems.ProfileGenerator;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Telemetry.Severity;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class Arm {

    // A couple notes about arm stuff
    // Shoulder encoder relative to ground, elbow encoder relative to bicep, wrist relative to forearm
    // Inverse kinematics are based on wrist joint pos
    // + X is out, +Y is up, (0,0) is the center of the bot & on the ground
    // theta=0 is straight out, theta=pi/2 is straight up, theta=-pi/2 is straight down

    public enum ArmControlMode{
        POSITION, TRANSLATE, NEUTRAL, PROFILE
    }

    public enum KinematicsMode {
        CONCAVE_UP, CONCAVE_DOWN
    }

    public enum ArmPosition {
        STARTING(new Vector2(4.75,8.5), Math.toRadians(45)),
        GROUND(new Vector2(27, 11), Math.toRadians(10)),
        PRIMED(new Vector2(15, 26), Math.toRadians(60)),
        SUBSTATION(new Vector2(28, 50.5), Math.toRadians(-2.5)),
        SUBSTATION_GRABBED(new Vector2(28, 50.5), Math.toRadians(10)),
        SUBSTATION_WAS_GRABBED(new Vector2(28, 50.5), Math.toRadians(-2.5)),

        //Cone Positions
        CONE_TOP(new Vector2(46, 60), Math.toRadians(60)),
        CONE_MIDDLE(new Vector2(30.75, 45), Math.toRadians(60)),
        CONE_BOTTOM(new Vector2(12, 10), 0),

        //Cube Positions
        CUBE_TOP(new Vector2(39, 50), 0.0),
        CUBE_MIDDLE(new Vector2(22.5, 35), 0.0),
        CUBE_BOTTOM(new Vector2(7.5, 18), 0.0),

        //Intermediate Positions
        GROUND_INTERMEDIATE(new Vector2(30, 30), Math.toRadians(45)),
        SCORE_INTERMEDIATE(new Vector2(22, 50), Math.toRadians(75));

        public final double[] angles;
        public final Vector2 position;
        public final double manipAng;

        ArmPosition(Vector2 pos, double manipAng) {
            position = pos;
            angles = calculateAngles(pos, manipAng);
            this.manipAng = manipAng;
        }

        // private ArmPosition(double shldrT, double elbowT, double manipAng){
        //     position = ;
        //     angles = new double[]{shldrT, elbowT, manipAng - (shldrT + elbowT)};
        // }

    }

    private static TalonFX  shldrMotor;
    private static TalonFX  elbowMotor;

    public static TalonSRX wristMotor;
    public static double wristTemp;
    private static CANCoder shldrCoder;
    private static CANCoder elbowCoder;
    public static CANCoder wristCoder;
    
    public static ArmControlMode mode;
    public static double[] destAngs = new double[3];
    private static Vector2 transVel; //translational
    public static double manipulatorAngle;
    private static KinematicsMode kinematicsMode = KinematicsMode.CONCAVE_UP;


    // Moved from RobotConfig
    // These values are needed at compile time for the ArmPosition enum
    public static final Vector2 shldrPos = new Vector2(7, 37);
    public static final double bicepLen   = 22.0;
    public static final double forearmLen = 26.0;
    public static final double manipLen   = 12.0;

    

    // Center of mass calculations
    // Aluminum tubing weight: https://www.easycalculation.com/engineering/mechanical/aluminum-rectangletube-weight.html
    // Note that these numbers are flawed. See the following list for things to check. (I know I should use Monday but idc):
    //   Are hex shafts negligible?            NOT CHECKED
    //   Is wrist mounting plate negligible?   NOT CHECKED
    //   Are talon SRX / wrist 775 negligible? NOT CHECKED
    //   Are chains/sprockets negligible?      NOT CHECKED
    // Right now I am assuming everything on that list is negligible
    // Offsets are the distances from the COMs to the previous joint. bicep->shoulder, forearm->elbow, etc.
    private static final double grav = -32.17;
    
    private static final double bicepWeight = 6;
    private static final double bicepCOMOffset = 15;
    private static final double bicepMass = bicepWeight/grav;
    private static final double bicepInertia = 512;

    private static final double forearmWeight = 2.86;
    private static final double forearmCOMOffset = 18;
    private static final double forearmMass = forearmWeight/grav;
    private static final double forearmInertia = 193;
    
    private static final double manipWeight = 8.3125;
    private static final double manipCOMOffset = 6;
    private static final double manipMass = manipWeight/grav;
    private static final double manipInertia = 0;

    public static double elbowAFFconst = 0.0002;
    public static double shldrAFFconst = 0.000325;
    public static double wristAFFconst = 0.003;//(6/5) * 2.8;

    // Shoulder PID
    public static double sP = 0.6 * 24.0 / 15.0;
    public static double sI = 0.0001 * 24.0 / 15.0;
    public static double sD = 0.1 * 24.0 / 15.0;

    // Elbow PID
    public static double eP = 0.6;
    public static double eI = 0.0001;
    public static double eD = 0.1;

    private static double armPosDead;
    private static double armTransDead;
    private static double armRotDead;

    public static double wristOffset = 0;

    public static void init(){
        shldrMotor = new TalonFX(RobotConfig.shldrMotorID);
        shldrCoder = new CANCoder(RobotConfig.shldrCoderID);
        
        shldrCoder.configFactoryDefault();
        shldrCoder.configFeedbackCoefficient(2 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond);
        double newPos = shldrCoder.getAbsolutePosition() - RobotConfig.shldrCoderOffset;
        if(newPos < -Math.PI){
            newPos += 2*Math.PI;
        } else if(newPos > Math.PI){
            newPos -= 2*Math.PI;
        }
        shldrCoder.setPosition(newPos);
        //Set the falcon to just use the CANCoder for PID
        shldrMotor.configRemoteFeedbackFilter(shldrCoder, 0);
        shldrMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,0,0);
        shldrMotor.setNeutralMode(NeutralMode.Brake);
        shldrMotor.configNominalOutputForward(0.01);
        shldrMotor.configNominalOutputReverse(0.01);
        shldrMotor.configNeutralDeadband(0.01);

        // shldrMotor.setInverted(true);
        shldrMotor.setSensorPhase(true);

        shldrMotor.config_kP(0, sP);
        shldrMotor.config_kI(0, sI);
        shldrMotor.config_kD(0, sD);
        shldrMotor.config_kF(0, 0);
        shldrMotor.configMotionCruiseVelocity(190);
		shldrMotor.configMotionAcceleration(300);
        
        // Clamp the output between -0.3 and 0.3
        shldrMotor.configClosedLoopPeakOutput(0, 1);

        elbowMotor = new TalonFX(RobotConfig.elbowMotorID);
        elbowCoder = new CANCoder(RobotConfig.elbowCoderID);

        elbowCoder.configFactoryDefault();
        //Encoder is PI/2 to -PI/2 radians
        elbowCoder.configFeedbackCoefficient(2 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond);
        elbowCoder.setPosition(elbowCoder.getAbsolutePosition() - RobotConfig.elbowCoderOffset);

        elbowMotor.configFactoryDefault();
        elbowMotor.setSensorPhase(true);
        elbowMotor.setInverted(true);
        //Set the falcon to just use the CANCoder for PID
        elbowMotor.configRemoteFeedbackFilter(elbowCoder, 0);
        elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,0,0);
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor.configNeutralDeadband(0.01);

        elbowMotor.config_kP(0, eP);
        elbowMotor.config_kI(0, eI);
        elbowMotor.config_kD(0, eD);
        elbowMotor.config_kF(0, 0);
        elbowMotor.configMotionCruiseVelocity(190);
		elbowMotor.configMotionAcceleration(300);

        // Clamp the output between -0.3 and 0.3
        elbowMotor.configClosedLoopPeakOutput(0, .5);

        wristMotor = new TalonSRX(RobotConfig.wristMotorID);
        wristMotor.configFactoryDefault();
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 0);
        double newWristPos = wristMotor.getSelectedSensorPosition(1) - 873;
        if(newWristPos < -Math.PI){
            newWristPos += 2*Math.PI;
        } else if(newWristPos > Math.PI){
            newWristPos -= 2*Math.PI;
        }
        wristMotor.setSelectedSensorPosition(newWristPos, 0, 0);

        

        // wristMotor = new TalonSRX(RobotConfig.wristMotorID);
        // wristMotor.configFactoryDefault();
        // wristMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        // wristMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        // wristCoder = new CANCoder(14);
        // wristCoder.configFactoryDefault();

        // // PI/2 to -PI/2 radians
        // wristCoder.configFeedbackCoefficient(2 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond);
        // wristCoder.setPosition(wristCoder.getAbsolutePosition() - RobotConfig.wristCoderOffset);

        // wristMotor.configRemoteFeedbackFilter(wristCoder, 0);
        // wristMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);
        
        wristMotor.setSensorPhase(true);
        //wristMotor.setInverted(true);
        wristMotor.config_kP(0, 0.9);
        wristMotor.config_kI(0, 0.0001);
        wristMotor.config_kD(0, 0.0);
        wristMotor.config_kF(0, 0);
        wristMotor.configMotionCruiseVelocity(269);
		wristMotor.configMotionAcceleration(420);

        armPosDead = RobotConfig.armPosDead;
        armTransDead = RobotConfig.armTransDead;
        armRotDead = RobotConfig.armRotDead;

        // For motion profiling
        shldrMotor.changeMotionControlFramePeriod(timeStepMS/2);
        elbowMotor.changeMotionControlFramePeriod(timeStepMS/2);
        wristMotor.changeMotionControlFramePeriod(timeStepMS/2);

        kinematicsMode = KinematicsMode.CONCAVE_UP;
        
        wristMotor.configVoltageCompSaturation(11); // Variable voltage compensation.
        wristMotor.enableVoltageCompensation(true); // Enable voltage compensation.

        wristMotor.configPeakCurrentLimit(30); // When we hit 30A
        wristMotor.configPeakCurrentDuration(100); //...for at least 100ms
        wristMotor.configContinuousCurrentLimit(20); //...we deploy a 20A continuous limit.
        wristMotor.enableCurrentLimit(true); // Enable current limit.

        setState(ArmControlMode.NEUTRAL, null, 0);
        enableBreak();
    }

    public static void disableBreak(){
        shldrMotor.setNeutralMode(NeutralMode.Coast);
        elbowMotor.setNeutralMode(NeutralMode.Coast);
        wristMotor.setNeutralMode(NeutralMode.Coast);
    }

    public static void enableBreak(){
        shldrMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }

    // ====  GETTERS  ====

    
    // Current angles of joints, in radians
    public static double getShldrTheta() { return -shldrCoder.getPosition(); }
    public static double getElbowTheta() { return elbowCoder.getPosition(); }
    public static double getWristTheta() { 
        return ticksToRad(wristMotor.getSelectedSensorPosition(0)) + wristOffset; 
    }
    public static double[] getJointAngles(){
        return new double[] {getShldrTheta(), getElbowTheta(), getWristTheta()};
    }

    // Destination angles of joints, in radians
    public static double getShldrDest() { return destAngs[0]; }
    public static double getElbowDest() { return destAngs[1]; }
    public static double getWristDest() { return destAngs[2]; }

    public static double getEeTheta()   { return getShldrTheta() + getElbowTheta() + getWristTheta(); }

    /**
     * Converts radians (0 pointing forward, counterclockwise) to encoder ticks
     */
    public static double radToTicks(double rad)   { return 4096.0 * rad / (2.0 * Math.PI); }
    /**
     * Converts encoder ticks to radians (0 pointing forward, counterclockwise)
     */
    public static double ticksToRad(double ticks) { return 2.0 * Math.PI * ticks / 4096.0; }
    /**
     * Converts from robot space (origin at the center of the robot's base) to arm space (origin at the shoulder)
     * @param robotPos a Vector2 in robot space, measured in inches
     * @return a Vector2 in arm space, measured in inches
     */
    public static Vector2 robotToArm(Vector2 robotPos) { return robotPos.sub(shldrPos); }

    // Calculating joint positions based on angles of consecutive joints
    public static Vector2 calcElbowPos(double shldrAng){
        return  getShldrPos().add(
                Vector2.fromPolar(shldrAng, bicepLen));
    }
    public static Vector2 calcWristPos(double shldrAng, double elbowAng){
        return  calcElbowPos(shldrAng).add(
                Vector2.fromPolar(shldrAng+elbowAng, forearmLen));
    }
    public static Vector2 calcManipTipPos(double shldrAng, double elbowAng, double wristAng){
        return  calcWristPos(shldrAng, elbowAng).add(
                Vector2.fromPolar(shldrAng+elbowAng+wristAng, manipLen));
    }

    // Returning the results of the functions above using encoder data
    public static Vector2 getShldrPos() { return shldrPos; }
    public static Vector2 getElbowPos(){ return calcElbowPos(getShldrTheta()); }
    public static Vector2 getWristPos(){ return calcWristPos(getShldrTheta(), getElbowTheta()); }
    public static Vector2 getManipTipPos(){ return calcManipTipPos(getShldrTheta(), getElbowTheta(), getWristTheta()); }

    // Calculating centers of mass
    public static Vector2 calcBicepCOM(double shldrAng){
        return getShldrPos().add(Vector2.fromPolar(shldrAng, bicepCOMOffset));
    }
    public static Vector2 getBicepCOM() {return calcBicepCOM(getShldrTheta()); }

    public static Vector2 calcForearmCOM(double shldrAng, double elbowAng){
        return calcElbowPos(shldrAng).add(Vector2.fromPolar(shldrAng+elbowAng, forearmCOMOffset));
    }
    public static Vector2 getForearmCOM() { return calcForearmCOM(getShldrTheta(), getElbowTheta()); }

    public static Vector2 calcManipCOM(double shldrAng, double elbowAng, double wristAng){
        return calcWristPos(shldrAng, elbowAng).add(Vector2.fromPolar(shldrAng+elbowAng+wristAng, manipCOMOffset));
    }
    public static Vector2 getManipCOM() { return calcManipCOM(getShldrTheta(), getElbowTheta(), getWristTheta()); }


    // ==== MATH ====

    /**
    * Calculate the destination angles of the motors given...
    * @param position wrist joint position
    * @param manipAng The desired angle (relative to the ground) of the manipulator.
    */
    public static double[] calculateAngles(Vector2 position, double manipAng){
        Vector2 pos = robotToArm(position);
        double elbowT = (kinematicsMode==KinematicsMode.CONCAVE_UP?1:-1 )* 
                                    Math.acos(  (Math.pow(pos.x,2) + Math.pow(pos.y,2) - Math.pow(bicepLen,2) - Math.pow(forearmLen,2)) /
                                    (2.0 * bicepLen * forearmLen) );
        double shldrT = Math.atan2(pos.y,pos.x) - 
                        Math.atan2(  (forearmLen * Math.sin(elbowT)),
                                    (bicepLen + forearmLen * Math.cos(elbowT)));
        double wristT = manipAng - (shldrT + elbowT);
        wristT = Math.max(Math.min(wristT, Math.PI/2),-Math.PI/2); //Safety clamp
        return new double[] {shldrT, elbowT, wristT};
    }

    /**
    * Calculate the angular velocities of the shoulder and elbow
    * @param vel The translational velocity of the wrist joint
    * @param posRobot The position of the wrist in robot space coordinates
    */
    public static double[] calculateAngVels(Vector2 vel, Vector2 posRobot){
        //See slack for math pdf
        Vector2 pos = robotToArm(posRobot);
        //This is basically just calculating the directional derivatives of the two angles
        double acosDeriv =  (kinematicsMode==KinematicsMode.CONCAVE_UP?1:-1) * 
                            (-1.0 / Math.sqrt(1 - Math.pow((Math.pow(pos.x,2) + Math.pow(pos.y,2) - Math.pow(bicepLen,2) - Math.pow(forearmLen,2)) /
                            (2.0 * bicepLen * forearmLen), 2)));
        double de_dx = acosDeriv * pos.x / (bicepLen * forearmLen);
        double de_dy = acosDeriv * pos.y / (bicepLen * forearmLen);
        
        double atanDeriv =  (bicepLen*forearmLen*Math.cos(getElbowTheta()) + Math.pow(forearmLen, 2)) / 
                            (Math.pow(bicepLen + forearmLen*Math.cos(getElbowTheta()), 2) + Math.pow(forearmLen*Math.sin(getElbowTheta()), 2));
        double ds_dx = (-pos.y / (Math.pow(pos.x, 2) + Math.pow(pos.y, 2))) - atanDeriv * de_dx;
        double ds_dy = ( pos.x / (Math.pow(pos.x, 2) + Math.pow(pos.y, 2))) - atanDeriv * de_dy;
        double sVel = vel.dot(new Vector2(ds_dx, ds_dy));
        double eVel = vel.dot(new Vector2(de_dx, de_dy));
        return new double[] {sVel, eVel};
    }

    /**
    * Calculate the angular velocities of the shoulder and elbow
    * @param vel The translational velocity of the wrist joint
    */
    public static double[] calculateAngVels(Vector2 vel){
        return calculateAngVels(vel, getWristPos());
    }


    // ==== Actual Control ====

    private static Vector2 destPos;
    /**
    * Set the state of the arm
    * @param mode Arm control mode, either POSITON or TRANSLATE
    * @param input The main movement data to apply. 
    *   In position mode: The X, Y location to move the wrist joint to. 
    *   In translate mode: The translational velocity for the wrist joint [-1,1].
    * @param manipAng The desired angle (relative to the ground) of the manipulator.
    */
    public static void setState(ArmControlMode mode, Vector2 input, double manipAng){
        stopProfile();
        Arm.mode = mode;
        switch(mode){
            case POSITION:
                destAngs = calculateAngles(input, manipAng);
                destPos = input;
                manipulatorAngle = manipAng;
                break;
            case TRANSLATE:
                transVel = input;
                manipulatorAngle = manipAng;
                break;
            case NEUTRAL:
                //Don't need to do anything. We're just going to use the AFFs
                break;
        }
    }
    public static void setStateNeutral(){
        setState(ArmControlMode.NEUTRAL, null, 0);
    }

    /**
    * Set the arm state to a preset
    * @param pos One of the preset positions
    */
    public static void setState(ArmPosition pos){
        stopProfile();
        // System.out.println("JDFKSHFJKHSDKF"+Thread.currentThread().getStackTrace()[2].getClassName());
        Arm.mode = ArmControlMode.POSITION;
        destPos = pos.position;
        destAngs = pos.angles.clone();
        manipulatorAngle = pos.angles[0] + pos.angles[1] + pos.angles[2];

        ArmStateController.currentState = pos;

        if (!goingToIntermediate)
            startTimer(pos, "POSITION (DIRECT)");
    }

    //set state of the arm but if its at the pos it moves to another stat
    public static boolean goingToIntermediate;
    private static ArmPosition nextPos;
    public static void setStateIntermediate(ArmPosition intPos, ArmPosition trueDestPos){
        stopProfile();
        goingToIntermediate = true;
        Arm.mode = ArmControlMode.POSITION;
        destPos = intPos.position;
        destAngs = intPos.angles;
        nextPos = trueDestPos;
        manipulatorAngle = trueDestPos.angles[0] + trueDestPos.angles[1] + trueDestPos.angles[2];

        ArmStateController.currentState = intPos;

        startTimer(trueDestPos, "POSITION (INTERMEDIATE)");
    }

    public static void setState(double shldrAng, double elbowAng, double wristAng){
        stopProfile();
        Arm.mode = ArmControlMode.POSITION;
        destAngs = new double[] {shldrAng, elbowAng, wristAng};
        manipulatorAngle = shldrAng + elbowAng + wristAng;
    }


    private static ProfileGenerator generationThread = null;
    private static boolean profileFinished = false;

    public static void setStateProfile(ArmPosition pos){
        Arm.mode = ArmControlMode.PROFILE;
        nextPos = pos;
        stopProfile();
        generationThread = new ProfileGenerator(getWristPos(), pos.position, pos.angles[2], 0.0007, 0.00012);
        profileFinished = false;
        shldrMotor.configMotionProfileTrajectoryPeriod(10);
        elbowMotor.configMotionProfileTrajectoryPeriod(10);
        wristMotor.configMotionProfileTrajectoryPeriod(10);
        shldrMotor.startMotionProfile(generationThread.shldrStream, 5, ControlMode.MotionProfile);
        elbowMotor.startMotionProfile(generationThread.elbowStream, 5, ControlMode.MotionProfile);
        wristMotor.startMotionProfile(generationThread.wristStream, 5, ControlMode.MotionProfile);
        generationThread.start();

        startTimer(pos, "PROFILE");
    }

    /**
     * Converts angular velocities of the two joints into translational velocity of the wrist joint
     * @param shldrAng position of shoulder (rad)
     * @param shldrAngVel angular velocity of shoulder (rad/s)
     * @param elbowAng position of elbow (rad)
     * @param elbowAngVel angular velocity of elbow (rad/s)
     * @return the translational velocity of the wrist (in/s)
    */
    public static Vector2 angToTransVel(double shldrAng, double shldrAngVel, double elbowAng, double elbowAngVel){
        return new Vector2(
            bicepLen*-Math.sin(shldrAng)*shldrAngVel - Math.sin(shldrAng + elbowAng)*forearmLen*(shldrAngVel + elbowAngVel),
            bicepLen* Math.cos(shldrAng)*shldrAngVel + Math.cos(shldrAng + elbowAng)*forearmLen*(shldrAngVel + elbowAngVel)
        );
    }
    
    final static int timeStepMS = 10;
    final static double timeConst = timeStepMS/1000.0;
    
    public static void stopProfile(){
        if(generationThread!=null){
            generationThread.interrupt();
            generationThread = null;
        }
        destAngs = new double[3];
        shldrMotor.clearMotionProfileTrajectories();
        elbowMotor.clearMotionProfileTrajectories();
        wristMotor.clearMotionProfileTrajectories();
    }

    public static double[] calculateAFFs(double shldrAng, double elbowAng, double wristAng){
        double[] AFFs = new double[3];
        double totalWeight = manipWeight;
        Vector2 COMpos = calcManipCOM(shldrAng, elbowAng, wristAng);
        // System.out.printf("Shldrpos: %s ", Arm.getBicepCOM());
        // System.out.printf("Elbowpos: %s ", Arm.getForearmCOM());

        // System.out.printf("Wristpos: %s\n", Arm.getManipCOM());
        AFFs[2] = totalWeight * COMpos.sub(calcWristPos(shldrAng, elbowAng)).x * wristAFFconst;
        // System.out.printf("wCOM: %s", COMpos);
        // The elbow needs to support both the manip and forearm
        // This is literally a weighted average
        COMpos = COMpos.mul(totalWeight).add(calcForearmCOM(shldrAng, elbowAng).mul(forearmWeight)).div(totalWeight+forearmWeight);
        totalWeight += forearmWeight;
        AFFs[1] = totalWeight * COMpos.sub(calcElbowPos(shldrAng)).x * elbowAFFconst;
		// System.out.printf("eCOM: %s ", COMpos);

        COMpos = COMpos.mul(totalWeight).add(calcBicepCOM(shldrAng).mul(bicepWeight)).div(totalWeight+bicepWeight);
        totalWeight += bicepWeight;
        AFFs[0] = bicepWeight * COMpos.sub(getShldrPos()).x * shldrAFFconst;
        return AFFs;
    }

    public static void update() {

        // updateWristCalibration();

        if (atPosition())
            endTimer();

        double[] AFFs = calculateAFFs(getShldrTheta(), getElbowTheta(), getWristTheta());

        // System.out.printf("sCOM: %s\n", COMpos);
        // System.out.printf("Shldr: %.2f Elbow: %.2f Wrist: %.2f\n", AFFs[0],AFFs[1],AFFs[2]);

        switch (mode){
            case POSITION:
                //We don't check the rules of the game and robot constraints because we have defined these as valid points
                if(goingToIntermediate){
                    if (destPos.sub(getWristPos()).mag()<12){
                        setState(nextPos);
                        goingToIntermediate = false;
                    }
                }
                shldrMotor.set(ControlMode.MotionMagic, radToTicks(destAngs[0]), DemandType.ArbitraryFeedForward, AFFs[0]);
                elbowMotor.set(ControlMode.MotionMagic, radToTicks(destAngs[1]), DemandType.ArbitraryFeedForward, AFFs[1]);
                wristMotor.set(ControlMode.MotionMagic, radToTicks(destAngs[2]-wristOffset), DemandType.ArbitraryFeedForward, AFFs[2]);

                if (RobotBase.isSimulation()) {
                    if (Math.abs(shldrCoder.getPosition() + destAngs[0]) > Math.PI * 2) {
                        shldrCoder.setPosition(-destAngs[0]);
                        elbowCoder.setPosition(destAngs[1]);
                        wristMotor.setSelectedSensorPosition((destAngs[2]-wristOffset) * 4096.0 / (Math.PI * 2)+(Math.PI * 2));
                    } else {
                        shldrCoder.setPosition(shldrCoder.getPosition() + (-destAngs[0] - shldrCoder.getPosition()) * 0.05);
                        elbowCoder.setPosition(elbowCoder.getPosition() + (destAngs[1] - elbowCoder.getPosition()) * 0.05);
                        wristMotor.setSelectedSensorPosition(wristMotor.getSelectedSensorPosition() + ((destAngs[2]-wristOffset) * 4096.0 / (Math.PI * 2) - wristMotor.getSelectedSensorPosition()) * 0.05);
                    }
                }
                break;


            case TRANSLATE:
                /*
                When in manual control there are a few constraints we need to impose.
                1 - The furthest point out must be within 48". (game rule)
                2 - The lowest point should not go into the ground. (self explanatory)
                Constraints 1 & 2 must be applied to the translational velocity
                Constraint 3 must be appplied to the rotational velocity of the shoulder. 
                  (This may lead to unpredictable impacts on the translation. This is ok. The operator shouldn't be doing much manual control up high)
                There are some potential errors with the way I have done this:
                  Does not account for the manipulator being a rectangle instead of a line
                  Does not account for the forearm structure that extends past wrist joint
                As such there is some padding so it limits to 46" & 2" above ground
                */
                
                // Linearly smooth movement into stop at 46"
                // If over 46" it should be negative & so should come back

                // double extension = Math.abs(manipulatorAngle)>=Math.PI/2 ? getWristPos().x : getManipTipPos().x - (RobotConfig.frameLength/2.0);
                // if(extension > 44.0){
                //     transVel.x = Math.min(transVel.x, -0.5*(extension-46));
                // }
                // double minY = Math.min(getWristPos().y, getManipTipPos().y);
                // if(minY < 4.0){
                //     transVel.y = Math.max(transVel.y, -0.5*(minY-2));
                // }

                double[] angVels = calculateAngVels(transVel);
                double m = Math.max(angVels[0], angVels[1]);
                if(m>1){ // normalize
                    angVels[0] /= m;
                    angVels[1] /= m;
                }
                angVels[0] *= 1; // Clamping
                angVels[1] *= 1;

                // Compensate for gear ratio (assumes elbow has higher than shoulder)
                final double shldrRatio = 75 * (28.0/12.0);
                final double elbowRatio = 75 * (28.0/15.0);
                double diff = shldrRatio/elbowRatio;
                angVels[0] *= diff;

                double wristT = manipulatorAngle - (getShldrTheta() + getElbowTheta());
                
                shldrMotor.set(ControlMode.PercentOutput, angVels[0], DemandType.ArbitraryFeedForward, AFFs[0]);
                elbowMotor.set(ControlMode.PercentOutput, angVels[1], DemandType.ArbitraryFeedForward, AFFs[1]);
                wristMotor.set(ControlMode.MotionMagic, wristT-wristOffset, DemandType.ArbitraryFeedForward, AFFs[2]);

                break;


            case PROFILE:
            System.out.printf("%.3f, %.3f, %.3f \n", shldrMotor.getActiveTrajectoryVelocity(), elbowMotor.getActiveTrajectoryVelocity(), wristMotor.getActiveTrajectoryVelocity());
                
                //resetting the I error once the profiles finish to prevent overshoor. All the profiles are the same length, so they should finish at the same time
                if(!profileFinished && shldrMotor.isMotionProfileFinished()){
                    profileFinished = false;
                    shldrMotor.setIntegralAccumulator(0.0);
                    elbowMotor.setIntegralAccumulator(0.0);
                    wristMotor.setIntegralAccumulator(0.0);
                    setState(nextPos);
                } else {
                    destAngs[0] = ticksToRad(shldrMotor.getActiveTrajectoryPosition());
                    destAngs[1] = ticksToRad(elbowMotor.getActiveTrajectoryPosition());
                    destAngs[2] = ticksToRad(wristMotor.getActiveTrajectoryPosition());
                    // System.out.println(destAngs[0]+","+destAngs[1]+","+destAngs[2]);
                }
                break;


            case NEUTRAL:
                //Only use the AFFs
                shldrMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, AFFs[0]);
                elbowMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, AFFs[1]);
                wristMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, AFFs[2]);
                break;
        }
    }

    /**
     * Recalibrates the wrist if the forward or reverse limit switch is closed, and rotates it upward if `recalibratingWrist` is true
     */
    // public static void updateWristCalibration() {
    //     if(Manipulator.motor.isFwdLimitSwitchClosed() == 1 || Manipulator.motor.isRevLimitSwitchClosed() == 1){
    //         wristOffset = Math.toRadians(90) - getWristTheta();
    //         recalibratingWrist = false;
    //     } else if (recalibratingWrist) {
    //         wristMotor.set(ControlMode.PercentOutput, 0.4);
    //     }
    // }

    /**
     * Returns whether or not the Arm is at its Destination
     * @return Whether or not the arm is at the right position
     */
    public static boolean atPosition() {
        if(goingToIntermediate)
            return false;
        switch(mode){
            case POSITION:
                return destPos.sub(getWristPos()).mag() < 5.;
            case TRANSLATE:
                return transVel.mag() < armTransDead;
            case PROFILE:
                return shldrMotor.isMotionProfileFinished() && elbowMotor.isMotionProfileFinished() && wristMotor.isMotionProfileFinished();
            default:
                return false;
        }
    }

    private static boolean recalibratingWrist;

    public static void recalibrateWrist(){
        recalibratingWrist = true;
    }

    // DEBUGGING
    private static boolean timerRunning = false;
    private static ArmPosition lastTarget = ArmPosition.STARTING;
    private static String startPosStr;
    private static String controlModeStr;
    private static double timerStartSeconds;

    public static void startTimer(ArmPosition targetPos, String controlMode) {
        timerRunning = true;
        controlModeStr = controlMode;

        if (lastTarget != null && lastTarget.position.sub(getWristPos()).mag() < 5.)
            startPosStr = lastTarget.toString();
        else 
            startPosStr = getWristPos().toString();

        lastTarget = targetPos;
        timerStartSeconds = RTime.now();
    }

    public static void endTimer() {
        if (!timerRunning)
            return;
        timerRunning = false;

        String message = String.format(
            "Arm movement from %s to %s using %s completed in %.2fs", 
            startPosStr, 
            lastTarget.toString(), 
            controlModeStr, 
            RTime.now() - timerStartSeconds
        );
        Telemetry.log(Severity.DEBUG, message);
        startPosStr = lastTarget.toString();
    }

}