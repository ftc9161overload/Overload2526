package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
// This is basically the same as the intake and outtake because they're all just one motor spinning
public class RotarySubsystem implements Subsystem {
    private final MotorEx motor = new MotorEx(UniConstants.ROTARY_MOTOR_STRING);

    private boolean isOn = false;
    private double motorSpeed = 0.2;
    private static double p = 0.6, d = 0, f = 0, l = 0.07;
    private PDFLControllerRadial mCon = new PDFLControllerRadial(p, d,f,l);
    private int currentChamber = 1;
    private double currentPosition = 0;
    private double targetPosition = 0;
    private final double ticksPerRotation = (537.7*170)/38;
    private double chamber1 = 2*Math.PI*1/3;
    private double chamber2 = 2*Math.PI*2/3;
    private double chamber3 = 2*Math.PI;

    private boolean halfChamber = false;
    private double chamberOffset = 0;

    // Constructor for building a Rotary Subsystem object
//    public RotarySubsystem(HardwareMap hMap, String motor) {
//        this.motor = hMap.get(DcMotorEx.class, motor);
//        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.motor.setMode(UniConstants.ROTARY_RUN_MODE);
//    }

    // Getter method for returning the isOn boolean
    public boolean getIsOn() {return isOn;}
    public boolean getHalfChamber() {return halfChamber;}
    public double getPosition() {
        return currentPosition;
    }
    public double getTargetPosition() {
        return targetPosition;
    }
    // Setter method for setting isOn to an input value
    public void setIsOn(boolean isOn) {
        this.isOn = isOn;
    }

    private void Chamber(int chamber) {
        if (chamber == 1) {
            targetPosition = chamber1;
            currentChamber = 1;
        }
        else if(chamber == 2) {
            targetPosition = chamber2;
            currentChamber = 2;
        }
        else if(chamber == 3) {
            targetPosition = chamber3;
            currentChamber = 3;
        }

    }

    public void previousChamber() {
        if(currentChamber == 1) {
            Chamber(3);
        }
        else if (currentChamber == 2) {
            Chamber(1);
        }
        else if (currentChamber == 3) {
            Chamber(2);
        }
    }
    public void nextChamber() {
        if (currentChamber == 1) {
            Chamber(2);
        }
        else if (currentChamber == 2) {
            Chamber(3);
        }
        else if (currentChamber == 3) {
            Chamber(1);
        }
    }

    public void setHalfChamber(boolean halfChamber) {this.halfChamber = halfChamber;}


    // Runs the motor if isOn is true
    @Override
    public void periodic() {
        if (halfChamber) {
            chamberOffset = Math.PI/3;
        } else {
            chamberOffset = 0;
        }
        currentPosition = ((motor.getCurrentPosition() % ticksPerRotation)/ticksPerRotation * 2 * Math.PI);

        if(isOn) {
            if(targetPosition > currentPosition) {
                mCon.setTarget(targetPosition + chamberOffset);
            }
            else {
                mCon.setTarget(targetPosition+Math.PI*2 + chamberOffset);
            }

            mCon.update(currentPosition);
            motor.setPower(mCon.runPDFL(0.01));
        }

    }
    public String debugText() {
        mCon.setPDFL(p,d,f,l);
        return "motorSpeed: " + motorSpeed + "\nPDFL: " + mCon.runPDFL(0.1) + "\nisOn: " + isOn + "\nCurrent Chamber: " + currentChamber + "\nCurrent Position: " + currentPosition + "\nTarget Position: " + targetPosition + "\nCurrent Offset: " + chamberOffset + "\n HalfChamber?: " + halfChamber;
    }

}
