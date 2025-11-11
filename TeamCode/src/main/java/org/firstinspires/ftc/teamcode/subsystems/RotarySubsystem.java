package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.PDFLController;
import dev.nextftc.core.subsystems.Subsystem;

@Configurable
// This is basically the same as the intake and outtake because they're all just one motor spinning
public class RotarySubsystem implements Subsystem {
    private final DcMotorEx motor;
    private boolean isOn = false;
    private double motorSpeed = 0.2;
    private PDFLController mCon;
    private int currentChamber = 1;
    private int currentPosition = 0;
    private int targetPosition = 0;
    private final double ticksPerRotation = (537.7*170)/38;
    private final int chamberTicks = (int) (ticksPerRotation/3);
    private int chamber1 = chamberTicks*1;
    private int chamber2 = chamberTicks*2;
    private int chamber3 = chamberTicks*3;

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

    public void setTargetPosition(int targetPosition) {this.targetPosition = targetPosition;}

    public void setTargetPositionDeg(int targetPosition) {this.targetPosition = (int) (targetPosition*ticksPerRotation/360);}

    public void setTargetPositionRad(int targetPosition) {this.targetPosition = (int) (targetPosition*ticksPerRotation/2/Math.PI);}

    public void Chamber(int chamber) {
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


    // Runs the motor if isOn is true
    @Override
    public void periodic() {
        if(isOn) {
            currentPosition = (int) (motor.getCurrentPosition() % ticksPerRotation);
            if(targetPosition > currentPosition) {
                mCon.setTarget(targetPosition);
            }
            else {
                mCon.setTarget(targetPosition+ticksPerRotation);
            }

            mCon.update(currentPosition);
            motor.setPower(mCon.runPDFL(1));
        }

    }
    public String debug() {
        return "motorSpeed: " + motorSpeed + "\nPDFL: " + mCon.runPDFL(0.1) + "\nisOn: " + isOn + "\nCurrent Chamber: " + currentChamber + "\nCurrent Position: " + currentPosition + "\nTarget Position: " + targetPosition;
    }

}
