package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Util.Vector2D;

import org.firstinspires.ftc.teamcode.Util.PDFLController;

public class SwervePodSubsystem {
    private CRServo servo;
    private double x,y, posOffset;
    private DcMotorEx motor;
    private double mPow;
    private double servoOffset, currentPos, targetPos, distCW, distCCW;
    private PDFLController sCon;
    AnalogInput sIn;

    public SwervePodSubsystem(double x, double y, String servo, String motor, String analogInput, HardwareMap hMap) {
        this.x = x;
        this.y = y;
        this.servo = hMap.get(CRServo.class, servo);
        this.motor = hMap.get(DcMotorEx.class, motor);
        this.sIn = hMap.get(AnalogInput.class, analogInput);
        posOffset = Math.atan2(this.x, this.y);
    }

    public void setServoOffset(double offset) {
        this.servoOffset = offset/360*3.3;
    }

    public void setTargetPos() {
        targetPos = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;
    }
    public void Update(Vector2D translational, Vector2D rotational) {


        Vector2D resultant = translational.add(rotational.rotate(posOffset + Math.PI/2));


        currentPos = (sIn.getVoltage() + servoOffset)%3.3;
//        targetPos = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;
        targetPos = resultant.angle()/2/Math.PI*3.3;
        distCW = 3.3 - Math.abs(currentPos - targetPos);
        distCCW = Math.abs(currentPos - targetPos);


        if (distCW < distCCW){
            sCon.setTarget(currentPos + distCW);
        }
        else {
            sCon.setTarget(currentPos - distCCW);
        }

        sCon.update(currentPos);
        servo.setPower(sCon.runPDFL(0.05));
        motor.setPower(resultant.magnitude());
    }
    public void Update(double x, double y, double rotation) {

        Vector2D resultant = new Vector2D(x, y).add( new Vector2D(rotation,0).rotate(posOffset + Math.PI/2));


        currentPos = (sIn.getVoltage() + servoOffset)%3.3;
//        targetPos = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;
        targetPos = resultant.angle()/2/Math.PI*3.3;
        distCW = 3.3 - Math.abs(currentPos - targetPos);
        distCCW = Math.abs(currentPos - targetPos);


        if (distCW < distCCW){
            sCon.setTarget(currentPos + distCW);
        }
        else {
            sCon.setTarget(currentPos - distCCW);
        }

        sCon.update(currentPos);
        servo.setPower(sCon.runPDFL(0.05));
        motor.setPower(resultant.magnitude());
    }

    public void setPDFL(double P, double D, double F, double L) {
        sCon.setPDFL(P, D, F, L);
    }


}
