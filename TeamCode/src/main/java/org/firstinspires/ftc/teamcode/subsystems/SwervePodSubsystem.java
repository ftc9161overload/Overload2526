package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.Vector2D;

import org.firstinspires.ftc.teamcode.Util.PDFLController;


@Configurable
public class SwervePodSubsystem {
    private CRServo servo;
    private double x,y, posOffset;
    private DcMotorEx motor;
    private double mPow;
    private double servoOffset, currentPos, targetPos;
    public static double p = .9, d = 0.03, f = 0, l = 0.03, errorMin = 0.07;
    private PDFLControllerRadial sCon = new PDFLControllerRadial(0.5, 0.0, 0.0, 0.1);
    public static boolean pdflUpdate = false;
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

    public void setTargetPos() {
        targetPos = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;
    }
    public void update(Vector2D translational, Vector2D rotational) {

        if (pdflUpdate) {
            sCon.setPDFL(p, d, f, l);
        }


        Vector2D resultant = translational.add(rotational.rotate(posOffset + Math.PI/2));


        currentPos = (sIn.getVoltage()/3.3*2*Math.PI) - Math.PI;
        targetPos = (resultant.angle()+servoOffset) % (2*Math.PI) - Math.PI;


       sCon.setTarget(targetPos);

        sCon.update(currentPos);
        servo.setPower(-sCon.runPDFL(errorMin));
        motor.setPower(resultant.magnitude());
    }

    public void update(double x, double y, double rotation) {

        update(new Vector2D(x, y), new Vector2D(rotation,0));
    }

    public void setPDFL(double p, double d, double f, double l) {
        SwervePodSubsystem.p = p; SwervePodSubsystem.d = d; SwervePodSubsystem.f = f; SwervePodSubsystem.l = l;
    }

    public String debugText() {
        return "Servo: " + sIn.getVoltage() + "\nCurrentPos: " + currentPos + "\ntargetPos: " + targetPos + "\nPDFL: "  + sCon.runPDFL(0.05) + "\n Offset: " + servoOffset + "\n\n" + sCon.debugText();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }


}
