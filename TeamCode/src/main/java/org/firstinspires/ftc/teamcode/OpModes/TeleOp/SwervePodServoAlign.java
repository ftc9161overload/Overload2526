package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;

@TeleOp(name = "SwervePodServoAlign", group = "TeleOp")
@Configurable
public class SwervePodServoAlign extends OpMode {
    private static SwerveDrivetrain swerveDrivetrain;

    @Override
    public void init() {
        swerveDrivetrain = new SwerveDrivetrain();
    }

    @Override
    public void loop() {
        swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);

        if (gamepad2.aWasPressed()){
            swerveDrivetrain = new SwerveDrivetrain();
        }

        telemetry.addData("FL", SwerveDrivetrain.flOffset);
        telemetry.addData("FR", SwerveDrivetrain.frOffset);
        telemetry.addData("BL", SwerveDrivetrain.blOffset);
        telemetry.addData("BR", SwerveDrivetrain.brOffset);
        telemetry.update();

    }
}
