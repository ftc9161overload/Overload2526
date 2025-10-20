package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.PDFLController;

public class SwervePodSubsystem {
    private CRServo servo;
    private DcMotorEx motor;
    private double mPow;
    private double servoOffset;
    private double currentPos;
    private double targetPos;
    private double distCW;
    private double distCCW;
    private PDFLController sCon;
    AnalogInput sIn;

    public SwervePodSubsystem(String servo, String motor, HardwareMap hMap) {
        this.servo = hMap.get(CRServo.class, servo);
        this.motor = hMap.get(DcMotorEx.class, motor);
    }
    
    public void setServoOffset(double offset) {
        this.servoOffset = offset/360*3.3;
    }
    public void setTargetPos(double targetPos) {
        this.targetPos = targetPos;
    }

    public void setMotorPower(double power) {
        this.mPow = power;
    }

    public void setTargetPos() {
        targetPos = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;
    }
    public void teleMove() {
        currentPos = (sIn.getVoltage() + servoOffset)%3.3;
        targetPos = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;
        distCW = 3.3 - Math.abs(currentPos - targetPos);
        distCCW = Math.abs(currentPos - targetPos);

        if (distCW < distCCW){
            sCon.setTarget(currentPos + distCW);
        }
        else {
            sCon.setTarget(currentPos - distCCW);
        }
        motor.setPower(mPow);
    }


}
