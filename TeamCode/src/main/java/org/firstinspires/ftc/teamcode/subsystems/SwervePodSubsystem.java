package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.Vector2D;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import com.pedropathing.math.*;


@Configurable
public class SwervePodSubsystem {
    private CRServo servo;
    private double x,y, posOffset;
    private DcMotorEx motor;
    private int motorDirection = 1; // positive 1 means normal but -1 means reverse
    private double mPow;
    private double servoOffset, currentPos, targetPos, flippedTargetPos;
    private double setTargetPos = 0;
    public static double p = .8, d = 0.01, f = 0, l = 0.1, errorMin = 0.07;
    private Timer flipTimer = new Timer();
    private static double flipCooldownSeconds = 0.2; // tweakable
    private PDFLControllerRadial sCon = new PDFLControllerRadial(0.5, 0.0, 0.0, 0.1);

    private UniConstants.swerveDriveType driveMode = UniConstants.swerveDriveType.TURN_GO;

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
        flippedTargetPos = MathUtil.piWraparound(targetPos + Math.PI);

        double diffTargetPos = Math.abs(MathUtil.piWraparound(targetPos-currentPos));
        double diffFlippedTargetPos = Math.abs(MathUtil.piWraparound(flippedTargetPos-currentPos));


        // Based on drivemode do different stuff
        switch (driveMode) {
            // Deadzone is just the boring not good drivemode but is reliable
            case DEADZONE:
                if(drive.magnitude() > UniConstants.deadzone) {
                    sCon.setTarget(targetPos);
                }

                sCon.update(currentPos);
                servo.setPower(-sCon.runPDFL(errorMin));

                motor.setPower(drive.magnitude());
                break;

            // Turn and Go is the better drivemode
            case TURN_GO:

                if (Double.isNaN(setTargetPos)) setTargetPos = currentPos;

                if (drive.magnitude() > UniConstants.deadzone) {

                    // Only pick new direction when pod isn't moving
                    if (motor.getVelocity() < UniConstants.servoMovementDeadzone) {

                        if (diffFlippedTargetPos < diffTargetPos && flipTimer.hasElapsedSeconds(flipCooldownSeconds)) {
                            // Flipping is faster
                            setTargetPos = flippedTargetPos;
                            motorDirection = -1;
                            flipTimer.reset();
                        } else if (diffFlippedTargetPos >= diffTargetPos && flipTimer.hasElapsedSeconds(flipCooldownSeconds)) {
                            // Normal path is faster
                            setTargetPos = targetPos;
                            motorDirection = 1;
                            flipTimer.reset();
                        }
                    }

                    sCon.setTarget(setTargetPos);
                }

                sCon.update(currentPos);
                servo.setPower(-sCon.runPDFL(errorMin));

                // MUST compute diff from FINAL chosen target!
                double diffFinal = Math.abs(MathUtil.piWraparound(setTargetPos - currentPos));

                // Only drive when aimed properly
                if (diffFinal <= UniConstants.radialDeadzone) {
                    motor.setPower(motorDirection * drive.magnitude());
                } else {
                    motor.setPower(0);
                }

                break;
        }
    }

    public void update(Vector2D translational, Vector2D rotational) {


        Vector2D resultant = getResultantVector(translational,rotational);
        update(resultant);
    }

    public void update(double x, double y, double rotation) {

        update(new Vector2D(x, y), new Vector2D(rotation,0));
    }


    public void setServoMode(UniConstants.swerveDriveType mode) {
        driveMode = mode;
    }

    public void setPDFL(double p, double d, double f, double l) {
        this.p = p; this.d = d; this.f = f; this.l = l;
        sCon.setPDFL(p,d,f,l);
    }

    public String debugText() {
        setPDFL(p,d,f,l);
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
