package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MidLevel.SwervePodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;

@TeleOp(name = "AlignPods", group = "TeleOp")
public class AlignPods extends OpMode {
    private static SwerveDrivetrain swerveDrivetrain;
    SwervePodSubsystem[] pods;

    @Override
    public void init() {
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);
        pods = swerveDrivetrain.getSwervePods();
    }

    @Override
    public void loop() {
        swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);
        swerveDrivetrain.setPosZero();
        swerveDrivetrain.setServoPowZero();

        if(gamepad1.aWasPressed()) {
            SwerveDrivetrain.flOffset = (int) pods[0].getAnalogInPos();
            SwerveDrivetrain.frOffset = (int) pods[1].getAnalogInPos();
            SwerveDrivetrain.blOffset = (int) pods[2].getAnalogInPos();
            SwerveDrivetrain.brOffset = (int) pods[3].getAnalogInPos();
        }

        telemetry.addData("FL currentPos: ", Math.toDegrees(pods[0].getAnalogInPos()));
        telemetry.addData("FR currentPos: ", Math.toDegrees(pods[1].getAnalogInPos()));
        telemetry.addData("BL currentPos: ", Math.toDegrees(pods[2].getAnalogInPos()));
        telemetry.addData("FR currentPos: ", Math.toDegrees(pods[3].getAnalogInPos()));

        telemetry.addData("current FL: ", SwerveDrivetrain.flOffset);
        telemetry.addData("current FR: ", SwerveDrivetrain.frOffset);
        telemetry.addData("current BL: ", SwerveDrivetrain.blOffset);
        telemetry.addData("current BR: ", SwerveDrivetrain.brOffset);
        telemetry.update();
    }
}
