package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubystem;
import org.firstinspires.ftc.teamcode.subsystems.RotarySubsystem;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "NextFTC Auton", group = "Auton")
@Configurable
public class NextFTCAuton extends NextFTCOpMode {


    private Timer timer = new Timer();
    public static double intakePower = 0.5;
    //public static double outtakePower = 10;
    public static double servoPos = 0.3; // 0.3 off, 0.8 on

    private boolean intaking = false;
    private boolean outtaking = false, transitioning = false;

    private boolean chamberOffset = true;

    private String userInterface = "";


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
        outtakeSubystem.set(true);
        outtakeSubystem.setVel(2000);
        outtaking=true;
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        timer.reset();

    }

    @Override
    public void onUpdate() {
        outtaking=true;


//
//        if (gamepad1.aWasPressed()) {
//            intaking = !intaking;
//        }
        if (intaking) {
            intakeSubystem.debug(intakePower);
        } else {
            intakeSubystem.debug(0);
            outtaking = false;
        }

//        if (gamepad1.yWasPressed() && !transitioning) {
//            chamberOffset = !chamberOffset;
//        }

        if (chamberOffset) {
            rotarySubsystem.OffsetHalfChamber();
        } else {
            rotarySubsystem.noOffset();
        }

//        if (gamepad1.bWasPressed()) {
//            outtaking = !outtaking;
//        }

        if (outtaking) {
            outtakeSubystem.set(true);
        } else {
            outtakeSubystem.set(false);
        }

//        if (gamepad1.aWasPressed() && !intaking){
//            intakeSubystem.debug(intakePower);
//            intaking = true;
//        } else if (gamepad1.aWasPressed() && intaking){
//            intakeSubystem.debug(0);
//            intaking = false;
//        }
//        if (gamepad1.b && !outtaking){
//            outtakeSubystem.set(true);
//        } else if (gamepad1.bWasPressed()){
//            outtakeSubystem.set(false);
//        }
//        if (gamepad1.xWasPressed() && !transitioning) {
//            rotarySubsystem.nextChamber();
//        }
//        if (gamepad1.yWasPressed() && !chamberOffset) {
//            rotarySubsystem.OffsetHalfChamber();
//            chamberOffset = true;
//        } else if (gamepad1.yWasPressed()) {
//            //rotarySubsystem.noOffset();
//            chamberOffset = false;
//        }
       // outtakeSubystem.setVel(outtakeSubystem.getTargetVel() + gamepad1.left_trigger * -10 + gamepad1.right_trigger * 10);
        outtakeSubystem.debugServo(servoPos);

//        if (gamepad1.dpad_left) {
//            servoPos = 0.3;
//            transitioning = false;
//        } else if (gamepad1.dpad_up) {
//            servoPos = 0.85;
//            transitioning = true;
//        }

        rotarySubsystem.setChamberOffset2(rotarySubsystem.getChamberOffset2() + (gamepad1.left_bumper ? -.005 : gamepad1.right_bumper ? .005 : 0) );


        if (timer.getTimeSeconds() <1) {
            swerveDrivetrain.simpleRunDrive(0, -1, 0);
        } else {
            swerveDrivetrain.simpleRunDrive(0,0,0);
        }
//        outtakeSubystem.setVel(outtakePower);


        telemetry.addData("gamepad1 y: ", gamepad1.y);
        telemetry.addData("chamberOffset", chamberOffset);
        telemetry.addData("intaking: ", intaking);
        telemetry.addData("outtaking: ", outtaking);
        telemetry.addData("Rotary Debug: ",rotarySubsystem.debugText());
        telemetry.addData("\n\nIntake Debug: ",intakeSubystem.debugText());
        telemetry.addData("\n\nOuttake Debug: ", outtakeSubystem.debugText());
        userInterface += "\nFlywheel Speed: " + outtakeSubystem.getVel() + " / " +outtakeSubystem.getTargetVel() + "\n";
////        for (int i = 0; i < 20; i++) {
//            userInterface += outtakeSubystem.getVel() / 2680 > i/20.0 ? "[]" : "-";
//        }

        telemetry.addLine(userInterface);
        telemetry.update();
        userInterface = "";

        outtakeSubystem.periodic();
        rotarySubsystem.periodic();
    }

}
