package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubystem;
import org.firstinspires.ftc.teamcode.subsystems.RotarySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UniConstants;

import dev.nextftc.ftc.NextFTCOpMode;
import kotlin.Unit;

@TeleOp(name = "NextFTC TeleOp", group = "TeleOp")
@Configurable
public class NextFTCTeleOp extends NextFTCOpMode {


    private static RotarySubsystem rotarySubsystem;
    private static IntakeSubystem intakeSubystem;
    private static OuttakeSubystem outtakeSubystem;
    private static SwerveDrivetrain swerveDrivetrain;

    @Override
    public void onInit() {
        rotarySubsystem = new RotarySubsystem(hardwareMap, UniConstants.ROTARY_MOTOR_STRING);
        intakeSubystem = new IntakeSubystem(UniConstants.INTAKE_MOTOR_STRING, hardwareMap);
        outtakeSubystem = new OuttakeSubystem(UniConstants.OUTTAKE_MOTOR_STRING, hardwareMap);
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {
        if (gamepad1.aWasReleased()){
            intakeSubystem.toggle();
        }
        if (gamepad1.bWasReleased()){
            outtakeSubystem.toggle();
        }
        if (gamepad1.yWasReleased()){
            rotarySubsystem.setIsOn(true);
        }
        if (gamepad1.xWasReleased()){
            rotarySubsystem.nextChamber();
        }

    }

}
