package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC TeleOp", group = "TeleOp")
@Configurable
public class NextFTCTeleOp extends NextFTCOpMode {

    //public static double outtakePower = 10;
    public static double servoPos = 0.3; // 0.3 off, 0.8 on
    private double movementScaler = 1.0;
    public static double outtakePreset1 = 1900;
    public static double outtakePreset2 = 2560;

    private String userInterface = "";

    private static LauncherSubsystem launcherSubsystem;
    //private static RotarySubsystem rotarySubsystem;
    //private static OuttakeSubsystem outtakeSubsystem;
    private static SwerveDrivetrain swerveDrivetrain;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(IntakeSubsystem.INSTANCE),
                BindingsComponent.INSTANCE
        );

        launcherSubsystem = new LauncherSubsystem(hardwareMap);
        launcherSubsystem.rotarySubsystem = new RotarySubsystem(hardwareMap, UniConstants.ROTARY_MOTOR_STRING);
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

        Gamepads.gamepad1().a()
                .whenTrue(IntakeSubsystem.INSTANCE.run)
                .whenFalse(IntakeSubsystem.INSTANCE.stop);


        if (gamepad1.yWasPressed() && !launcherSubsystem.outtakeSubsystem.getTransitioning()) {
            launcherSubsystem.rotarySubsystem.setHalfChamber(!launcherSubsystem.rotarySubsystem.getHalfChamber());
            launcherSubsystem.stateUpdate(0);
        }

        if (gamepad1.bWasPressed()) {
            launcherSubsystem.outtakeSubsystem.toggle();
            launcherSubsystem.stateUpdate(0);
        }

        if (gamepad1.rightBumperWasPressed() && !launcherSubsystem.outtakeSubsystem.getTransitioning()) {
            launcherSubsystem.rotarySubsystem.nextChamber();
            launcherSubsystem.stateUpdate(0);
        }
        if(gamepad1.leftBumperWasPressed() && !launcherSubsystem.outtakeSubsystem.getTransitioning()) {
            launcherSubsystem.rotarySubsystem.previousChamber();
            launcherSubsystem.stateUpdate(0);
        }

        if(gamepad1.bWasPressed() && !launcherSubsystem.getStart()) {
            launcherSubsystem.outtakeSubsystem.toggle();

        }
        launcherSubsystem.outtakeSubsystem.setVel(launcherSubsystem.outtakeSubsystem.getTargetVel() + gamepad1.left_trigger * -10 + gamepad1.right_trigger * 10);

        if(gamepad1.leftStickButtonWasReleased()) {
            launcherSubsystem.outtakeSubsystem.setVel(outtakePreset1);
        }
        if(gamepad1.rightStickButtonWasReleased()) {
            launcherSubsystem.outtakeSubsystem.setVel(outtakePreset2);
        }

        if(gamepad1.startWasPressed() || gamepad2.dpad_up){
            launcherSubsystem.setShootCount(3);
            launcherSubsystem.stateUpdate(1);
        } else if (gamepad2.dpad_right || gamepad2.dpad_left) {
            launcherSubsystem.setShootCount(2);
            launcherSubsystem.stateUpdate(1);

        } else if (gamepad2.dpad_down) {
            launcherSubsystem.setShootCount(1);
            launcherSubsystem.stateUpdate(1);

        } else if(gamepad2.xWasPressed()) {
            launcherSubsystem.setShootCount(0);
            launcherSubsystem.stateUpdate(1);
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

        if (gamepad2.rightBumperWasPressed()){
            movementScaler = 0.5;
            launcherSubsystem.stateUpdate(0);
        }
        if (gamepad2.leftBumperWasPressed()){
            movementScaler = 1;
            launcherSubsystem.stateUpdate(0);
        }

        swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, movementScaler);

//        telemetry.addData("gamepad1 y: ", gamepad1.y);
//        telemetry.addData("chamberOffset", chamberOffset);
//        telemetry.addData("intaking: ", intaking);
//        telemetry.addData("outtaking: ", outtaking);
//        telemetry.addData("Rotary Debug: ",launcherSubsystem.rotarySubsystem.debugText());
//        telemetry.addData("\n\nIntake Debug: ",intakeSubystem.debugText());
//        telemetry.addData("\n\nOuttake Debug: ", launcherSubsystem.outtakeSubsystem.debugText());
//        userInterface += "\nFlywheel Speed: " + outtakeSubystem.getVel() + " / " +outtakeSubystem.getTargetVel() + "\n";
//        userInterface += "\n nudge amount: " + ro[tarySubsystem.getChamberOffset2();
//        userInterface += "\n" + rotarySubsystem.debugText();
//        for (int i = 0; i < 20; i++) {
//            userInterface += outtakeSubystem.getVel() / 2680 > i/20.0 ? "[]" : "-";
//        }
//        telemetry.addLine(launcherSubsystem.debugText());
//        telemetry.addLine(userInterface);
//        telemetry.addLine(swerveDrivetrain.debugString());
        telemetry.update();
        userInterface = "";

        launcherSubsystem.update();
    }

}
