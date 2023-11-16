package frc.robot.subsystems;

import java.util.LinkedList;
import com.ctre.phoenix.motion.TrajectoryPoint;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.utils.Vector2;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

public class ProfileGenerator extends Thread{
    /**This list is used for testing the math*/
    public LinkedList<double[]> jointProfile;
    public LinkedList<Vector2> wristProfile;
    public BufferedTrajectoryPointStream shldrStream, elbowStream, wristStream;
    public Vector2 iPos, fPos;
    public double fManipAng;
    public double maxAccel, maxVel;
    private Vector2 dir;
    private double totalDist, brakingDist;
    private boolean isTriangular;


    public ProfileGenerator(Vector2 initialPos, Vector2 finalPos, double finalManipAng, double maxAccel, double maxVel){
        iPos = initialPos;
        fPos = finalPos;
        fManipAng = finalManipAng;
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.setDaemon(true);

        shldrStream = new BufferedTrajectoryPointStream();
        elbowStream = new BufferedTrajectoryPointStream();
        wristStream = new BufferedTrajectoryPointStream();
        jointProfile = new LinkedList<>();
        wristProfile = new LinkedList<>();

        Vector2 displacement = fPos.sub(iPos);
        totalDist = displacement.mag();
        brakingDist = totalDist - (Math.pow(maxVel, 2) / (2 * maxAccel));
        dir = displacement.norm();
        isTriangular = brakingDist > totalDist / 2.0;
    }

    public ProfileGenerator(ArmPosition initialPos, ArmPosition finalPos, double maxAccel, double maxVel){
        this(initialPos.position, finalPos.position, finalPos.angles[2], maxAccel, maxVel);
    }

    @Override
    public void run(){
        //variables used for finding the angles
        double vel = 0.0;
        double accel = maxAccel;
        Vector2 pos = new Vector2(iPos.x, iPos.y);

        //other variables
        double[] lastAngles = Arm.calculateAngles(iPos, 0.0);
        double[] affs;
        double[] angles;
        TrajectoryPoint[] points;

        while(pos.sub(iPos).mag() < totalDist){
            //killing the thread if interrupted
            if(Thread.interrupted()) return;
            
            if(isTriangular){
                if(pos.sub(iPos).mag() > totalDist/2.0){
                    accel = -maxAccel;
                }
                if(vel < 0.0){
                    break;
                }
            }else{
                if(vel > maxVel){
                    accel = 0.0;
                }
                if(pos.sub(fPos).mag() < brakingDist){
                    accel = -maxAccel;
                }
                if(vel < 0.0){
                    break;
                }
                
            }
            //updating the wrist state
            vel += accel;
            pos = pos.add(dir.mul(vel));

            //creating trajectory points. angles should be in radians
            angles = Arm.calculateAngles(pos, 0.0);
            affs = Arm.calculateAFFs(angles[0], angles[1], fManipAng);

            points = createPoints(angles, lastAngles, affs, false);
            
            //writing to the buffer
            shldrStream.Write(points[0]);
            elbowStream.Write(points[1]);
            wristStream.Write(points[2]);

            lastAngles = angles;

            //adding to the unnessecary lists
            jointProfile.add(angles);
            wristProfile.add(pos);
        }

        angles = Arm.calculateAngles(pos, 0.0);
        affs = Arm.calculateAFFs(angles[0], angles[1], fManipAng);

              //this is a hack \/\/\/
        points = createPoints(angles, angles, affs, true);
        
        shldrStream.Write(points[0]);
        elbowStream.Write(points[1]);
        wristStream.Write(points[2]);

        jointProfile.add(angles);
        wristProfile.add(pos);
    }

    private TrajectoryPoint[] createPoints(double[] pos, double[] lastPos, double[] affs, boolean isLast){
        TrajectoryPoint[] points = new TrajectoryPoint[]{new TrajectoryPoint(), new TrajectoryPoint(), new TrajectoryPoint()};
        for(int i = 0; i < 3; i++){
            points[i].useAuxPID = false;
            points[i].isLastPoint = isLast;
            points[i].velocity = 10.0*Arm.radToTicks(pos[i] - lastPos[i]);
            points[i].position = Arm.radToTicks(pos[i]);
            points[i].arbFeedFwd = affs[i];
        }
        return points;
    }
    
}
