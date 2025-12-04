package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotarySubsystem;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC TeleOp", group = "TeleOp")
@Configurable
public class NextFTCTeleOp extends NextFTCOpMode {

    //public static double outtakePower = 10;
    public static double servoPos = 0.3; // 0.3 off, 0.8 on

    private boolean chamberOffset = true;

    private String userInterface = "";


    private static RotarySubsystem rotarySubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static OuttakeSubsystem outtakeSubsystem;
    private static SwerveDrivetrain swerveDrivetrain;

    @Override
    public void onInit() {
        rotarySubsystem = new RotarySubsystem(hardwareMap, UniConstants.ROTARY_MOTOR_STRING);
        intakeSubsystem = new IntakeSubsystem(UniConstants.INTAKE_MOTOR_STRING, hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(UniConstants.OUTTAKE_MOTOR_STRING, UniConstants.OUTTAKE_SERVO_STRING,hardwareMap);
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

        if (gamepad1.aWasPressed()) {
            intakeSubsystem.set(!intakeSubsystem.get());
        }

        if (gamepad1.yWasPressed() && !outtakeSubsystem.getTransitioning()) {
            rotarySubsystem.setHalfChamber(!chamberOffset);
        }



        if (gamepad1.bWasPressed()) {
            outtakeSubsystem.set(!outtakeSubsystem.get());
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
        if (gamepad1.xWasPressed() && !outtakeSubsystem.getTransitioning()) {
            rotarySubsystem.nextChamber();
        }
//        if (gamepad1.yWasPressed() && !chamberOffset) {
//            rotarySubsystem.OffsetHalfChamber();
//            chamberOffset = true;
//        } else if (gamepad1.yWasPressed()) {
//            //rotarySubsystem.noOffset();
//            chamberOffset = false;
//        }
        outtakeSubsystem.setVel(outtakeSubsystem.getTargetVel() + gamepad1.left_trigger * -10 + gamepad1.right_trigger * 10);

        if (gamepad1.dpad_left) {
            outtakeSubsystem.setServo(UniConstants.engagementLevel.OFF);
        } else if (gamepad1.dpad_up) {
            outtakeSubsystem.setServo(UniConstants.engagementLevel.ON);
        } else if (gamepad1.dpad_right) {
            outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_ON);
        } else if (gamepad1.dpad_down) {
            outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_OFF);
        }

//        rotarySubsystem.setChamberOffset2(rotarySubsystem.getChamberOffset2() + (gamepad1.left_bumper ? -.005 : gamepad1.right_bumper ? .005 : 0) );


        swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);
        //outtakeSubystem.setVel(outtakePower);


//        telemetry.addData("gamepad1 y: ", gamepad1.y);
//        telemetry.addData("chamberOffset", chamberOffset);
//        telemetry.addData("intaking: ", intaking);
//        telemetry.addData("outtaking: ", outtaking);
        telemetry.addData("Rotary Debug: ",rotarySubsystem.debugText());
//        telemetry.addData("\n\nIntake Debug: ",intakeSubystem.debugText());
        telemetry.addData("\n\nOuttake Debug: ", outtakeSubsystem.debugText());
//        userInterface += "\nFlywheel Speed: " + outtakeSubystem.getVel() + " / " +outtakeSubystem.getTargetVel() + "\n";
//        userInterface += "\n nudge amount: " + rotarySubsystem.getChamberOffset2();
//        userInterface += "\n" + rotarySubsystem.debugText();
////        for (int i = 0; i < 20; i++) {
//            userInterface += outtakeSubystem.getVel() / 2680 > i/20.0 ? "[]" : "-";
//        }

        telemetry.addLine(userInterface);
        telemetry.update();
        userInterface = "";

        outtakeSubsystem.periodic();
        rotarySubsystem.periodic();
        intakeSubsystem.periodic();
    }

}
