package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Servo Position & Motor Power Control", group = "TeleOp")
public class ServoPositionMotorControl extends OpMode {

    // Servos
    private Servo s1, s2, s3;
    // Motors
    private DcMotor m1, m2, m3;

    // Predefined servo positions (0.0 to 1.0)
    private final double S1_POS = 0.2;
    private final double S2_POS = 0.7;
    private final double S3_POS = 1.0;

    // Predefined motor powers (-1.0 to 1.0)
    private final double M1_POWER = 0.8;
    private final double M2_POWER = -0.6;
    private final double M3_POWER = 0.3;

    @Override
    public void init() {
        // Map hardware
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");

        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");

        telemetry.addLine("Servos and Motors Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Set servo positions
        s1.setPosition(S1_POS);
        s2.setPosition(S2_POS);
        s3.setPosition(S3_POS);

        // Set motor powers
        m1.setPower(M1_POWER);
        m2.setPower(M2_POWER);
        m3.setPower(M3_POWER);

        // Telemetry for debugging
        telemetry.addData("Servo 1 Pos", S1_POS);
        telemetry.addData("Servo 2 Pos", S2_POS);
        telemetry.addData("Servo 3 Pos", S3_POS);

        telemetry.addData("Motor 1 Power", M1_POWER);
        telemetry.addData("Motor 2 Power", M2_POWER);
        telemetry.addData("Motor 3 Power", M3_POWER);

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
