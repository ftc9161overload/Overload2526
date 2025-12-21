package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeFlipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheelSubsystem;
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
                new SubsystemComponent(IntakeSubsystem.INSTANCE, LauncherSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        // INTAKE (HOLD TO USE)
        Gamepads.gamepad1().a()
                .whenBecomesTrue(IntakeSubsystem.INSTANCE.run)
                .whenBecomesFalse(IntakeSubsystem.INSTANCE.stop);

        // SET HALF ON THE ROTARY AND MAKES SURE FLIPPER IS NOT IN THE WAY
        Gamepads.gamepad1().y().toggleOnBecomesTrue()
                .whenBecomesTrue(LauncherSubsystem.INSTANCE.setHalfOn)
                .whenBecomesFalse(LauncherSubsystem.INSTANCE.setHalfOff);

        // OUTTAKE WHEEL STUFF
        Gamepads.gamepad1().x()
                .whenBecomesTrue(OuttakeWheelSubsystem.INSTANCE.turnOff);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(OuttakeWheelSubsystem.INSTANCE.setSpeedHigher);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(OuttakeWheelSubsystem.INSTANCE.setSpeedLower);

        // CHANGE ROTARY POSITION
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(RotarySubsystem.INSTANCE.nextChamber);

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(RotarySubsystem.INSTANCE.previousChamber);

        // AUTO LAUNCH 1 ARTIFACT
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(LauncherSubsystem.INSTANCE.Launch1);

        Gamepads.gamepad1().circle().toggleOnBecomesTrue()
                .whenBecomesTrue(OuttakeFlipperSubsystem.INSTANCE.setFullOn)
                .whenBecomesFalse(OuttakeFlipperSubsystem.INSTANCE.setFullOff);
    }

    @Override
    public void onUpdate() {

        telemetry.addData("FPS", timer.getTime()/ Math.pow(10.0,9));
        telemetry.addData("Rotary", RotarySubsystem.INSTANCE.debugText());
        telemetry.update();
        timer.reset();

        //swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, movementScaler);
    }

}
