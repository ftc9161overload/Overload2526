package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.Vector2D;

import org.firstinspires.ftc.teamcode.Util.PDFLController;


@Configurable
public class SwervePodSubsystem {
    private CRServo servo;
    private double x,y, posOffset;
    private DcMotorEx motor;
    private double mPow;
    private double servoOffset, currentPos, targetPos, distCW, distCCW;
    public static double p = .5, d = 0, f = 0, l = 0.1;
    private PDFLController sCon = new PDFLController(0.5, 0.0, 0.0, 0.1);
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

    public void setServoOffset(double offset) {
        this.servoOffset = offset/360*3.3;
    }

    public void setTargetPos() {
        targetPos = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;
    }
    public void update(Vector2D translational, Vector2D rotational) {

        if (pdflUpdate) {
            sCon.setPDFL(p, d, f, l);
        }


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
        servo.setPower(-sCon.runPDFL(0.05));
        motor.setPower(resultant.magnitude());
    }

    public void update(double x, double y, double rotation) {

        update(new Vector2D(x, y), new Vector2D(rotation,0));
    }

    public void setPDFL(double p, double d, double f, double l) {
        SwervePodSubsystem.p = p; SwervePodSubsystem.d = d; SwervePodSubsystem.f = f; SwervePodSubsystem.l = l;
    }

    public String debugText() {
        return "Servo: " + sIn.getVoltage() + "\nCurrentPos: " + currentPos + "\ntargetPos: " + targetPos + "\ndistCW: " + distCW + "\ndistCCW: " + distCCW + "\nPDFL: " + sCon.runPDFL(0.05) + "\n Offset: " + servoOffset + "\n\nCWTarget: " + (currentPos + distCW) + "\nCCWTarget: " + (currentPos - distCCW);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }


}
