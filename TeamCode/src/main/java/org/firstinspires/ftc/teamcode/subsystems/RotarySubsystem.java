package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

// This is basically the same as the intake and outtake because they're all just one motor spinning
public class RotarySubsystem implements Subsystem {
    private final DcMotorEx motor;
    private boolean isOn = false;
    private double motorSpeed = 0.5;

    // Constructor for building a Rotary Subsystem object
    public RotarySubsystem(HardwareMap hMap, String motor) {
        this.motor = hMap.get(DcMotorEx.class, motor);
    }

    // Getter method for returning the isOn boolean
    public boolean getIsOn() {return isOn;}

    // Setter method for setting isOn to an input value
    public void setIsOn(boolean isOn) {
        this.isOn = isOn;
    }

    // Setter method for setting motorSpeed to an input value
    public void setMotorSpeed(double motorSpeed) {
        this.motorSpeed = motorSpeed;
    }

    // Runs the motor if motorSpeed is true
    @Override
    public void periodic() {
        if(isOn) {
            motor.setPower(motorSpeed);
        }
    }
}
