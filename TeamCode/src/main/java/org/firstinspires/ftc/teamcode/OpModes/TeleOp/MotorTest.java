package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test OpMode", group = "Test")
public class MotorTest extends OpMode {

    private DcMotor testMotor;

    @Override
    public void init() {
        // Change "motor" to your configuration name
        testMotor = hardwareMap.get(DcMotor.class, "motor");

        // Reset encoder
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        testMotor.setPower(gamepad1.right_stick_x);

        // Output encoder value
        telemetry.addData("Encoder Position", testMotor.getCurrentPosition());
        telemetry.addData("Motor Power", testMotor.getPower());
        telemetry.update();
    }
}