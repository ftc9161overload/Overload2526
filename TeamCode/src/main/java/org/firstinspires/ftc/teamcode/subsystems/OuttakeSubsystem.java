package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.subsystems.Subsystem;

@Configurable
public class OuttakeSubsystem implements Subsystem {
    private boolean isOn = false;
    private final DcMotorEx motor;
    private final Servo servo;
    public static double fullOff = 0.3, off = 0.5, on = 0.8, fullOn = 0.85;
    private double servoPos = 0.3;
    private double motorPower = 0.8;
    private double targetVel = 1;
    private boolean transitioning = false;
    public static double p = 0.0001, d = 0.000001, f = 0, l = 0;
    private PDFLController mCon = new PDFLController(p,d,f,l);

    public OuttakeSubsystem(String motor, String servo, HardwareMap hMap){
        this.motor = hMap.get(DcMotorEx.class, motor);
        this.servo = hMap.get(Servo.class, servo);
    }

    public boolean getTransitioning() {
        return transitioning;
    }

    public void debugServo(double pos) {
        servo.setPosition(pos);
    }

    // Sets the position of the servo depending on what type is needed
    public void setServo(UniConstants.engagementLevel type) {
        if(type == UniConstants.engagementLevel.FULL_OFF) {
            servoPos = fullOff; // Fully Unengaged
            transitioning = false;
        }
        else if(type == UniConstants.engagementLevel.OFF) {
            servoPos = off; // Unengaged
            transitioning = true;
        }
        else if(type == UniConstants.engagementLevel.ON) {
            servoPos = on; // Engaged
            transitioning = true;
        }
        else if(type == UniConstants.engagementLevel.FULL_ON) {
            servoPos = fullOn; // Fully Engaged
            transitioning = true;
        }
    }
    // Sets isOn to the new value (Setter)
    public void set(boolean isOn) {
        this.isOn = isOn;
    }

    // Returns the current state of isOn (Getter)
    public boolean get() {
        return isOn;
    }

    // Toggles the isOn bool
    public void toggle() {
        this.isOn = !isOn;
    }

    public void debug(double power) {
        motor.setPower(power);
    }

    public double getServoPos() {return servoPos;}

    public void setVel(double vel ) {
        targetVel = vel;
    }
    public double getVel() {return motor.getVelocity();}
    public double getTargetVel() { return targetVel;}

    // Runs the motor if isOn is True with the motorSpeed
    @Override
    public void periodic() {
        if (isOn){
            mCon.setTarget(targetVel);
            mCon.update(motor.getVelocity());
            motor.setPower(motorPower += mCon.runPDFL(.1));

        }
        else {
            motor.setPower(0);
            motorPower = 0;
        }

        servo.setPosition(servoPos);


        motorPower = Math.max(-1, Math.min(1,motorPower));
    }
    public String debugText() {
        mCon.setPDFL(p,d,f,l);
        return "motorSpeed: " + motorPower + "\nPDFL: " + mCon.runPDFL(0.1) + "\nisOn: " + isOn + "\nMotor Val: " + motor.getVelocity() + "\nTarget Vel: " + targetVel;
    }
}
