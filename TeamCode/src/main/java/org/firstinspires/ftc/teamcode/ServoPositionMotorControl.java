package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.PDFLController;

@TeleOp(name = "Servo Position & Motor Power Control", group = "TeleOp")
public class ServoPositionMotorControl extends OpMode {

    // Servos
    private CRServo s1, s2, s3;
    // Motors
    private DcMotor m1, m2, m3;

    private PDFLController s1Con;
    private PDFLController s2Con;
    private PDFLController s3Con;

    // Predefined servo positions (0.0 to 1.0)
    private double S1_POS = 0.2;
    private double S2_POS = 0.7;
    private double S3_POS = 1.0;

    // Predefined motor powers (-1.0 to 1.0)
    private double M1_POWER = 0.8;
    private double M2_POWER = -0.6;
    private double M3_POWER = 0.3;

    private double distCW;
    private double distCW2;
    private double distCCW2;
    private double distCCW;
    private double currentPos;

    private double currentPos2;
    private double targetPos2;
    private double targetPos;

    private final double S1Offset = (double) 30/360*3.3;
    private final double S2Offset = (double) 330/360*3.3;

    AnalogInput S1_In;
    AnalogInput S2_In;
    AnalogInput S3_In;


    @Override
    public void init() {
        // Map hardware
        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
//        s3 = hardwareMap.get(Servo.class, "s3");

        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
//        m3 = hardwareMap.get(DcMotor.class, "m3");

        S1_In = hardwareMap.get(AnalogInput.class, "s1_In");
        S2_In = hardwareMap.get(AnalogInput.class, "s2_In");
//        S3_In = hardwareMap.get(AnalogInput.class, "S3_In");

        s1Con = new PDFLController(0.5, 0.0, 0.0, 0.1);
        s2Con = new PDFLController(0.5, 0.0, 0.0, 0.1);


        telemetry.addLine("Servos and Motors Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        currentPos = (S1_In.getVoltage() + S1Offset)%3.3;
        targetPos = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;

        currentPos2 = (S2_In.getVoltage() + S2Offset)%3.3;
        targetPos2 = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)/2/Math.PI*3.3;

        distCW = 3.3 - Math.abs(currentPos - targetPos);
        distCCW = Math.abs(currentPos - targetPos);

        distCW2 = 3.3 - Math.abs(currentPos2 - targetPos2);
        distCCW2 = Math.abs(currentPos2 - targetPos2);

        if (distCW < distCCW){
            telemetry.addLine("CW < CCW");
            s1Con.setTarget(currentPos + distCW);
        } else {
            s1Con.setTarget(currentPos - distCCW);
        }

        if(distCW2 < distCCW2) {
            telemetry.addLine("CW2 < CCW2");
            s2Con.setTarget(currentPos2 + distCW2);
        } else {
            s2Con.setTarget(currentPos2 - distCCW2);
        }


        s1Con.update(currentPos );
        s2Con.update(currentPos2 );

        s1.setPower(-s1Con.runPDFL(0.05));
        s2.setPower(-s2Con.runPDFL(0.05));
//        s1.setPower(0);
//        s2.setPower(0);


        //s3.setPosition(S3_POS);

        // Set motor powers
        m1.setPower(M1_POWER);
        m2.setPower(M1_POWER);
        //m3.setPower(M3_POWER);

        S1_POS = S1_In.getVoltage();
        S2_POS = S2_In.getVoltage();
        //S3_POS = S3_In.getVoltage();

        M1_POWER = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        M2_POWER = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        //M3_POWER = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

        S1_POS = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        S2_POS = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x);
        //S3_POS = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);



        // Telemetry for debugging
        telemetry.addData("Servo 1 Pos", currentPos);
        telemetry.addData("Servo 2 Pos", currentPos2);
//        telemetry.addData("Servo 3 Pos", S3_POS);

        telemetry.addData("Servo 1 Input", S1_In.getVoltage());
        telemetry.addData("Servo 2 Input", S2_In.getVoltage());

        telemetry.addData("Servo 1 Target", targetPos);
        telemetry.addData("Servo 2 Target", targetPos2);

        telemetry.addData("Servo 1 Controller output: ", s1Con.runPDFL(0.05));
        telemetry.addData("Servo 2 Controller output: ", s2Con.runPDFL(0.05));
        telemetry.addData("S1 Controller target: ", s1Con.getTarget());
        telemetry.addData("S2 Controller target: ", s2Con.getTarget());
        //telemetry.addData("Servo 3 Input", S3_In.getVoltage());

        telemetry.addData("Counter Clockwise distance 1: ", distCCW);
        telemetry.addData("Counter Clockwise distance 2: ", distCCW2);
        telemetry.addData("Clockwise distance 1: ", distCW);
        telemetry.addData("Clockwise distance 2: ", distCW2);

        telemetry.addData("Motor 1 Power", M1_POWER);
        telemetry.addData("Motor 2 Power", M2_POWER);
//        telemetry.addData("Motor 3 Power", M3_POWER);


        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop motors (servos hold position automatically)
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
    }
}
