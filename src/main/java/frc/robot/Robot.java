// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmControlMode;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.subsystems.swerve.SwervePID;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.subsystems.swerve.TipProtection;
import frc.robot.trajectory.Trajectory;
import frc.robot.trajectory.Waypoint;
import frc.robot.utils.AutoSelection;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

// Comp repo test


public class Robot extends TimedRobot {
	public static boolean teleauto = false;
	public static Joystick testFlightstick;
	public Trajectory traj = new Trajectory(
			new Vector2(-19.39, -279), 
			new Vector2(-22.39, -47.36), 
			new Waypoint(new Vector2(-40, -150), Math.PI / 2, null), 
			new Waypoint(new Vector2(-22.39, -75), Math.PI / 2, Waypoint.actions.GROUND), 
			0.3
			);
	public static boolean auto;

	@Override
	public void robotInit() {
		AutoSelection.setup();
		RobotConfig.init();
		Vision.init();
		BannerLight.init();
		Pigeon.init();
		Pigeon.zero();
		SwerveManager.init();
		SwervePosition.init();
		SwervePID.init();
		Arm.init();
		Manipulator.init();
		AutoBalancer.init();
		Scoring.init();
		OI.init();
		Telemetry.init();
		TipProtection.init(false);
		Pigeon.setYaw(270);
		Vision.setLimelightLED(false);
        Telemetry.log(Telemetry.Severity.INFO, "ROBOT", "Started Robot.");
		auto = false;
	}

	@Override
	public void robotPeriodic() {
		Pigeon.update();
		Manipulator.updateTemps();
		Telemetry.update(false);
        RTime.updateAbsolute();
	}

	@Override
	public void autonomousInit() {
		Manipulator.neutral();
		Vision.setLimelightLED(true);
		Arm.enableBreak();
		RTime.init();
		Pigeon.setYaw(270);
		// SwerveManager.pointWheels(0);
		//AutoSelection.run();
		SwervePosition.disableVision();
		auto = true;
	}

	@Override
	public void autonomousPeriodic() {
		RTime.update();
		SwervePosition.update();
		//Auto.update();
		SwerveManager.rotateAndDrive(traj.updateDest());
		Arm.update();
		Manipulator.update();
	}
	@Override 
	public void autonomousExit(){
		Vision.setLimelightLED(false);
		auto = false;
	}

	@Override
	public void teleopInit() {
		Arm.enableBreak();
		Scoring.stop();
		OI.init();
		if(Auto.endingArmPosition == null)
			Arm.setState(ArmControlMode.NEUTRAL, null, 0.0);
		else
			Arm.setState(Auto.endingArmPosition);
		SwervePosition.enableVision();
	}

	@Override
	public void teleopPeriodic() {
		RTime.update();
		SwervePosition.update();

		if (teleauto) {
			Auto.update();
			if (OI.driverStick.getRawButtonPressed(OI.balance)){
				teleauto = false;
			}
		} else {
			OI.joystickInput();
		}
		
		Arm.update();
		Manipulator.update();
		BannerLight.updateTeleop(); 
	}

	@Override
	public void teleopExit(){
		//Arm.disableBreak(); // Only at end of match, not after auto.
	}

	@Override
	public void testInit() {
		Arm.enableBreak();
		testFlightstick = new Joystick(3);
		// Arm.setState(Arm.ArmPosition.PRIMED);
		Arm.setState(ArmControlMode.NEUTRAL, null, 0.0);
		// Sensing.init();
		SwerveManager.pointWheels(0);
	}

	@Override
	public void testPeriodic() {
		Manipulator.update();
		// OI.operatorInput();
		// BannerLight.updateTeleop();
		// if (OI.driverStick.getRawButton(LogitechF310.BUTTON_A)) {
		// 	// Sensing.init();
		// 	// Sensing.distanceSensor.setEnabled(false);
		// 	// Telemetry.log(Severity.DEBUG, "Distance sensor disabled");
		// } else {
		// 	// Sensing.distanceSensor.setEnabled(true);
		// 	// Telemetry.log(Severity.DEBUG, "Distance sensor enabled");
		// 	try {
		// 		Telemetry.log(Severity.INFO, "Distance: " + Sensing.getDistance(Unit.kInches));
		// 	} catch (Exception e) {
		// 		Telemetry.log(Severity.WARNING, "Distance sensor invalid");
		// 	}
		// }

		// Manipulator.setCubeMode();
		Arm.update();
	}

	@Override
	public void testExit() {
		Arm.disableBreak();
	}

	@Override
	public void disabledInit(){
	}

    @Override
	public void disabledPeriodic() {
		SwervePosition.updateAveragePosVision();
		// SwervePosition.updateAverageRotVision();
		BannerLight.setTagInView(Vision.hasTarget(), AutoSelection.getSelected());
    }

	@Override
	public void simulationInit() {
        SwervePosition.setPosition(Scoring.getScoringTarget(4));
        Pigeon.setSimulatedRot(Math.PI * 3 / 2);
    }

	@Override
	public void simulationPeriodic() {}
}
