package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Timer;
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
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "NextFTC TeleOp", group = "TeleOp")
@Configurable
public class NextFTCTeleOp extends NextFTCOpMode {

    private double movementScaler = 1.0;
    public static double outtakePreset1 = 1900;
    public static double outtakePreset2 = 2560;

    private Timer timer = new Timer();

    private static LauncherSubsystem launcherSubsystem;
    private static SwerveDrivetrain swerveDrivetrain;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(IntakeSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        launcherSubsystem = new LauncherSubsystem(hardwareMap);
        launcherSubsystem.rotarySubsystem = new RotarySubsystem(hardwareMap, UniConstants.ROTARY_MOTOR_STRING);
        launcherSubsystem.outtakeSubsystem = new OuttakeSubsystem();
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);


        launcherSubsystem.rotarySubsystem.setIsOn(true);
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().a()
                .whenBecomesTrue(IntakeSubsystem.INSTANCE.run)
                .whenBecomesFalse(IntakeSubsystem.INSTANCE.stop);
    }

    @Override
    public void onUpdate() {

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

        telemetry.addData("FPS", timer.getTime()/ Math.pow(10.0,9));
        telemetry.update();
        timer.reset();

        swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, movementScaler);

        launcherSubsystem.update();
    }

}
