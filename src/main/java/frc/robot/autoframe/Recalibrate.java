package frc.robot.autoframe;

import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Telemetry.Severity;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class Recalibrate extends AutoFrame {
    private double duration;
    private double stopTime;
    private boolean correctPosition;
    private boolean correctRotation;
    private Vector2 positionSum = new Vector2(0,0);
    private double rotationSum = 0;
    private int posSamplesTaken;
    private int rotSamplesTaken;
    
    /**
     * Blocking. Recalibrates SwervePosition with data from the camera. Works by taking the average of all 
     * camera readings over a set duration, meaning a longer duration will translate to a more accurate reading. 
     * @param correctPosition whether or not we should use vision to correct SwervePosition
     * @param correctRotation whether on not we should use vision to correct the Pigeon
     * @param duration the amount of time the robot should spend recalibrating, in seconds.
     */
    public Recalibrate(boolean correctPosition, boolean correctRotation, double duration) {
        this.blocking = true;
        this.duration = duration;
        this.correctPosition = correctPosition;
        this.correctRotation = correctRotation;
    }

    @Override
    public void start() {
        this.stopTime = RTime.now() + duration;
    }

    @Override
    public void update() {
        if (correctPosition) {
            try {
                Vector2 pos = Vision.getRobotPos();
                positionSum = positionSum.add(pos);
                posSamplesTaken++;
                Vector2 calibratedPos = positionSum.div(posSamplesTaken);
                SwervePosition.setPosition(calibratedPos);
            } catch (Exception e) {}
            Telemetry.log(Severity.INFO, (posSamplesTaken == 0 ? "No samples taken" : "position: " + positionSum.div(posSamplesTaken)));
        }
        if (correctRotation) {
            try {
                double rot = Vision.getRobotYaw();
                rotationSum = rotationSum + rot;
                rotSamplesTaken++;
                double calibratedRot = rotationSum / rotSamplesTaken;
                Pigeon.setYaw(calibratedRot);
            } catch (Exception e) {}
            Telemetry.log(Severity.INFO, (rotSamplesTaken == 0 ? "No samples taken" : "rotation: " + rotationSum / rotSamplesTaken));
        }
        if (RTime.now() > this.stopTime)
            this.done = true;
    }

    @Override
    public void finish() {
        if (posSamplesTaken == 0 && correctPosition)
            Telemetry.log(Telemetry.Severity.WARNING, "Unable to recalibrate SwervePosition using vision.");
        if (rotSamplesTaken == 0 && correctRotation)
            Telemetry.log(Telemetry.Severity.WARNING, "Unable to recalibrate Pigeon using vision.");
    }
}
