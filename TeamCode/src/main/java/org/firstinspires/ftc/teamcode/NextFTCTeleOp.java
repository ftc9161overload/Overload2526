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

    public static double intakePower = 0.5;
    public static double outtakePower = 0.1;
    public static double servoPos = 0; // 0.3 off, 0.8 on

    private boolean intaking = false;
    private boolean outtaking = false;
    private boolean chamberOffset = true;


    private static RotarySubsystem rotarySubsystem;
    private static IntakeSubystem intakeSubystem;
    private static OuttakeSubystem outtakeSubystem;
    private static SwerveDrivetrain swerveDrivetrain;

    @Override
    public void onInit() {
        rotarySubsystem = new RotarySubsystem(hardwareMap, UniConstants.ROTARY_MOTOR_STRING);
        intakeSubystem = new IntakeSubystem(UniConstants.INTAKE_MOTOR_STRING, hardwareMap);
        outtakeSubystem = new OuttakeSubystem(UniConstants.OUTTAKE_MOTOR_STRING, UniConstants.OUTTAKE_SERVO_STRING,hardwareMap);
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);

        rotarySubsystem.setIsOn(true);
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {


    }

    @Override
    public void onUpdate() {


        if (gamepad1.aWasPressed()){
            intakeSubystem.toggle();
        }
        if (gamepad1.b && !outtaking){
            outtakeSubystem.set(true);
        } else if (gamepad1.bWasPressed()){
            outtakeSubystem.set(false);
        }
        if (gamepad1.xWasPressed()) {
            rotarySubsystem.nextChamber();
        }
        if (gamepad1.yWasPressed() && !chamberOffset) {
            rotarySubsystem.OffsetHalfChamber();
            chamberOffset = true;
        } else if (gamepad1.yWasPressed()) {
            rotarySubsystem.noOffset();
            chamberOffset = false;
        }

        intakeSubystem.debug(intakePower);
        outtakeSubystem.setVel(outtakeSubystem.getTargetVel() + gamepad1.left_trigger * -10 + gamepad1.right_trigger * 10);
        outtakeSubystem.debugServo(servoPos);

        if (gamepad1.dpad_left) {
            servoPos = 0.3;
        } else if (gamepad1.dpad_up) {
            servoPos = 0.8;
        }


        swerveDrivetrain.simpleRunDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        outtakeSubystem.setVel(outtakePower);


        telemetry.addData("Rotary Debug: ",rotarySubsystem.debugText());
        telemetry.addData("\nIntake Debug: ",intakeSubystem.debugText());
        telemetry.addData("\nOuttake Debug: ", outtakeSubystem.debugText());
        telemetry.update();

        outtakeSubystem.periodic();
        rotarySubsystem.periodic();
    }

}
