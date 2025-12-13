package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC TeleOp", group = "TeleOp")
@Configurable
public class NextFTCTeleOp extends NextFTCOpMode {

    //public static double outtakePower = 10;
    public static double servoPos = 0.3; // 0.3 off, 0.8 on
    private double movementScaler = 1.0;

    private String userInterface = "";

    private static LauncherSubsystem launcherSubsystem;
    //private static RotarySubsystem rotarySubsystem;
    private static IntakeSubsystem intakeSubsystem;
    //private static OuttakeSubsystem outtakeSubsystem;
    private static SwerveDrivetrain swerveDrivetrain;

    @Override
    public void onInit() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap);
        launcherSubsystem.rotarySubsystem = new RotarySubsystem(hardwareMap, UniConstants.ROTARY_MOTOR_STRING);
        intakeSubsystem = new IntakeSubsystem(UniConstants.INTAKE_MOTOR_STRING, hardwareMap);
        launcherSubsystem.outtakeSubsystem = new OuttakeSubsystem(UniConstants.OUTTAKE_MOTOR_STRING, UniConstants.OUTTAKE_SERVO_STRING,hardwareMap);
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);


        launcherSubsystem.rotarySubsystem.setIsOn(true);
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
            intakeSubsystem.toggle();
            launcherSubsystem.stateUpdate(0);
        }

        if (gamepad1.yWasPressed() && !launcherSubsystem.outtakeSubsystem.getTransitioning()) {
            launcherSubsystem.rotarySubsystem.setHalfChamber(!launcherSubsystem.rotarySubsystem.getHalfChamber());
            launcherSubsystem.stateUpdate(0);
        }

        if (gamepad1.bWasPressed()) {
            launcherSubsystem.outtakeSubsystem.toggle();
            launcherSubsystem.stateUpdate(0);
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
        if (gamepad1.xWasPressed() && !launcherSubsystem.outtakeSubsystem.getTransitioning()) {
            launcherSubsystem.rotarySubsystem.nextChamber();
            launcherSubsystem.stateUpdate(0);
        }

//        if (gamepad1.yWasPressed() && !chamberOffset) {
//            rotarySubsystem.OffsetHalfChamber();
//            chamberOffset = true;
//        } else if (gamepad1.yWasPressed()) {
//            //rotarySubsystem.noOffset();
//            chamberOffset = false;
//        }
        if(!launcherSubsystem.getStart()) {
            launcherSubsystem.outtakeSubsystem.setVel(launcherSubsystem.outtakeSubsystem.getTargetVel() + gamepad1.left_trigger * -10 + gamepad1.right_trigger * 10);
        }

        if(gamepad1.rightBumperWasPressed()) {
            launcherSubsystem.setShootCount(3);
        }

        if (gamepad1.dpad_left) {
            launcherSubsystem.outtakeSubsystem.setServo(UniConstants.engagementLevel.OFF);
            launcherSubsystem.stateUpdate(0);
        } else if (gamepad1.dpad_up) {
            launcherSubsystem.outtakeSubsystem.setServo(UniConstants.engagementLevel.ON);
            launcherSubsystem.stateUpdate(0);
        } else if (gamepad1.dpad_right) {
            launcherSubsystem.outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_ON);
            launcherSubsystem.stateUpdate(0);
        } else if (gamepad1.dpad_down) {
            launcherSubsystem.outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_OFF);
            launcherSubsystem.stateUpdate(0);
        }

//        rotarySubsystem.setChamberOffset2(rotarySubsystem.getChamberOffset2() + (gamepad1.left_bumper ? -.005 : gamepad1.right_bumper ? .005 : 0) );

        /*   Added a slow movement mode to allow for more precise control of the bot in play   */
        if (gamepad2.rightBumperWasPressed()){
            movementScaler = 0.5;
            launcherSubsystem.stateUpdate(0);
        }
        if (gamepad2.leftBumperWasPressed()){
            movementScaler = 1;
            launcherSubsystem.stateUpdate(0);
        }

        swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, movementScaler);
        //outtakeSubystem.setVel(outtakePower);


//        telemetry.addData("gamepad1 y: ", gamepad1.y);
//        telemetry.addData("chamberOffset", chamberOffset);
//        telemetry.addData("intaking: ", intaking);
//        telemetry.addData("outtaking: ", outtaking);
        telemetry.addData("Rotary Debug: ",launcherSubsystem.rotarySubsystem.debugText());
//        telemetry.addData("\n\nIntake Debug: ",intakeSubystem.debugText());
        telemetry.addData("\n\nOuttake Debug: ", launcherSubsystem.outtakeSubsystem.debugText());
//        userInterface += "\nFlywheel Speed: " + outtakeSubystem.getVel() + " / " +outtakeSubystem.getTargetVel() + "\n";
//        userInterface += "\n nudge amount: " + rotarySubsystem.getChamberOffset2();
//        userInterface += "\n" + rotarySubsystem.debugText();
////        for (int i = 0; i < 20; i++) {
//            userInterface += outtakeSubystem.getVel() / 2680 > i/20.0 ? "[]" : "-";
//        }
        telemetry.addLine(launcherSubsystem.debugText());
        telemetry.addLine(userInterface);
        telemetry.addLine(swerveDrivetrain.debugString());
        telemetry.update();
        userInterface = "";

        intakeSubsystem.periodic();
        launcherSubsystem.update();
    }

}
