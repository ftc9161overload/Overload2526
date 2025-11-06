package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp(name = "Cole", group = "TeleOp")
public class oneMotor extends OpMode {
    private DcMotorEx motor;
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "flm");
    }
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
    }
}
