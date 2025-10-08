package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "CR Servo Joystick Control", group = "TeleOp")
public class CRServoJoystickControl extends OpMode {

    private CRServo crServo;

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, "armServo");
        telemetry.addLine("Continuous Rotation Servo Initialized");
        telemetry.addLine("Use left joystick up/down to control speed & direction");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Joystick value ranges from -1.0 (full back) to 1.0 (full forward)
        double power = -gamepad1.left_stick_y; // invert so up = forward

        // Apply directly to CR servo power
        crServo.setPower(power);

        telemetry.addData("Joystick", gamepad1.left_stick_y);
        telemetry.addData("Servo Power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        crServo.setPower(0);
    }
}
