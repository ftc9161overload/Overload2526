package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubystem;
import org.firstinspires.ftc.teamcode.subsystems.RotarySubsystem;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC TeleOp", group = "TeleOp")
@Configurable
public class NextFTCTeleOp extends NextFTCOpMode {

    public static double intakePower = 0.1;
    public static double outtakePower = 0.1;


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
        if (gamepad1.a){
            intakeSubystem.debug(intakePower);
        } else  {
            intakeSubystem.debug(0);
        }
        if (gamepad1.b){
            outtakeSubystem.set(true);
        } else {
            outtakeSubystem.set(false);
        }

        outtakeSubystem.setVel(outtakePower);
//        if (gamepad1.x) {
//            rotarySubsystem.de
//        }

        outtakeSubystem.periodic();
    }

}
