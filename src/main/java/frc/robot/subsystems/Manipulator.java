// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.utils.Piece;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Manipulator {

    public static Compressor compressor;
    public static Solenoid solenoid;

    public static CANSparkMax motor;

    public static boolean isAtPoint;

    public static double power, intakeTemp, intakeCurrent;

    public static boolean autoGrabbing;

    public static void init() {
        // Pneumatics
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

        // NEO motor
        // We do NOT invert the NEO/SparkMax
        // Otherwise we can get some very real sparks...
        motor = new CANSparkMax(1, MotorType.kBrushless);
        power = 0;
    }

    public static void update() {
        isAtPoint = compressor.getPressureSwitchValue();
        motor.set(power);
    }

    public static void updateTemps() { 
        Arm.wristTemp = Arm.wristMotor.getTemperature(); 
        intakeTemp = motor.getMotorTemperature();
    }

    public static void setCubeMode() {
        // Sets solenoid to cube intaking position
        solenoid.set(true);
        autoGrabbing = false;
    }

    public static void setOpen() {
        // Sets solenoid to cube intaking position
        solenoid.set(true);
        autoGrabbing = false;
    }

    public static void setConeMode() {
        solenoid.set(false);
        autoGrabbing = true;
    }

    public static void coneModeAutomatic() {
        if (autoGrabbing) {
            setConeMode();
            return;
        }
        try {
            // Telemetry.log(Severity.DEBUG, "Distance: " + Sensing.getDistance(Unit.kInches));
            if (Arm.wristMotor.isRevLimitSwitchClosed() == 0) {
                if(ArmStateController.currentState==ArmPosition.SUBSTATION) {
                    Arm.destAngs[2] = ArmPosition.SUBSTATION.angles[2] + Math.toRadians(60);
                }
                autoGrabbing = true;
                // Telemetry.log(Severity.DEBUG, "Auto grabbing");
                return;
            }
        } catch (Exception e) {}
        setOpen();
    }

    public static Piece getMode() {
        return solenoid.get() ? Piece.CUBE : Piece.CONE;
    }

    
    public static void intake() {
        double intake = .30;
        power += (intake - power) * 0.5;
    }

    public static void outtake() {
        double outtake = -.38;
        power += (outtake - power) * 0.5;
    }

    public static void neutral() {
        double neutral = .10;
        power += (neutral - power) * 0.5;
    }

    public static void off() {
        double off = 0;
        power += (off - power) * 0.5;
    }

}

