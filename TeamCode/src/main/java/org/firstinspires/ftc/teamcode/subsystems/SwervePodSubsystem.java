package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.Vector2D;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import com.pedropathing.math.*;


@Configurable
public class SwervePodSubsystem {
    private CRServo servo;
    private double x,y, posOffset;
    private DcMotorEx motor;
    private double mPow;
    private double servoOffset, currentPos, targetPos;
    private double p = .7, d = 0.005, f = 0, l = 0.03, errorMin = 0.07;
    private PDFLControllerRadial sCon = new PDFLControllerRadial(0.5, 0.0, 0.0, 0.1);

    AnalogInput sIn;

    public SwervePodSubsystem(double x, double y, String servo, String motor, String analogInput, HardwareMap hMap) {
        this.x = x;
        this.y = y;
        this.servo = hMap.get(CRServo.class, servo);
        this.motor = hMap.get(DcMotorEx.class, motor);
        this.sIn = hMap.get(AnalogInput.class, analogInput);
        posOffset = Math.atan2(this.x, this.y);
    }

    public void setServoOffsetDeg(double offset) {
        this.servoOffset = offset/360*2*Math.PI;
    }
    public void setServoOffsetRad(double offset) {this.servoOffset = offset;}


    public double getRotationOffset() {return posOffset;}

    public Vector2D getResultantVector(Vector2D translational, Vector2D rotational) {
        return translational.add(rotational.rotate(posOffset + Math.PI/2));
    }

    public void update(Vector2D drive) {
        currentPos = (sIn.getVoltage() / 3.3 * 2 * Math.PI) - Math.PI;
        targetPos = (drive.angle() + servoOffset) % (2 * Math.PI) - Math.PI;


        if(drive.magnitude() > UniConstants.deadzone) {

            sCon.setTarget(targetPos);
        }


        sCon.update(currentPos);


        servo.setPower(-sCon.runPDFL(errorMin));
        motor.setPower(drive.magnitude());
    }

    public void update(Vector2D translational, Vector2D rotational) {


        Vector2D resultant = getResultantVector(translational,rotational);
        update(resultant);
    }

    public void update(double x, double y, double rotation) {

        update(new Vector2D(x, y), new Vector2D(rotation,0));
    }

    public void setPDFL(double p, double d, double f, double l) {
        this.p = p; this.d = d; this.f = f; this.l = l;
        sCon.setPDFL(p,d,f,l);
    }

    public String debugText() {
        return "Servo: " + sIn.getVoltage() +
                "\nCurrentPos: " + currentPos +
                "\ntargetPos: " + targetPos +
                "\nPDFL: "  + sCon.runPDFL(0.05) +
                "\n Offset: " + servoOffset +
                "\n\n" + sCon.debugText();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setMotorMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }
    public void setMotorDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

}
