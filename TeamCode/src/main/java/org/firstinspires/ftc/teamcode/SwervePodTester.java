package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SwervePodSubsystem;


@TeleOp(name = "Swerve Pod Tester", group = "TeleOp")
public class SwervePodTester extends OpMode {

    private SwervePodSubsystem[] pods;

    @Override
    public void init() {
        SwervePodSubsystem fl = new SwervePodSubsystem(-156.0, 156.0, "fls", "flm", "flsai", hardwareMap); // Front Left
        SwervePodSubsystem br = new SwervePodSubsystem(156.0, -156.0, "brs", "brm", "brsai", hardwareMap); // Back Right
        pods = new SwervePodSubsystem[]{fl, br};
    }

    @Override
    public void loop() {
        for (SwervePodSubsystem pod : pods) {
            pod.Update(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        }

        telemetry.addData("LX", gamepad1.left_stick_x);
        telemetry.addData("LY", gamepad1.left_stick_y);
        telemetry.addData("RX", gamepad1.right_stick_x);
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
