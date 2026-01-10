package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.Vector2D;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeFlipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "TeleOp", group = "TeleOp")
@Configurable
public class NextFTCTeleOp extends NextFTCOpMode {
    public NextFTCTeleOp() {
        addComponents(
            new SubsystemComponent(IntakeSubsystem.INSTANCE, LauncherSubsystem.INSTANCE, OuttakeWheelSubsystem.INSTANCE,Odometry.INSTANCE, Follower.INSTANCE),
            BulkReadComponent.INSTANCE,
            BindingsComponent.INSTANCE
        );
    }
    private double movementScaler = 1.0;
    public static double outtakePreset1 = 1900;
    public static double outtakePreset2 = 2560;

    private Timer timer = new Timer();

    private static LauncherSubsystem launcherSubsystem;
    private static SwerveDrivetrain swerveDrivetrain;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(IntakeSubsystem.INSTANCE, LauncherSubsystem.INSTANCE, OuttakeWheelSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        OuttakeWheelSubsystem.INSTANCE.targetSpeed = 0;
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);

        Odometry.INSTANCE.initReal();

        //RotarySubsystem.INSTANCE.resetOffset();
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        RotarySubsystem.INSTANCE.locked = false;
        // INTAKE (HOLD TO USE)
        Gamepads.gamepad1().a()
                .whenBecomesTrue(IntakeSubsystem.INSTANCE.run)
                .whenBecomesFalse(IntakeSubsystem.INSTANCE.stop);

        Gamepads.gamepad2().a()
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
                .whenBecomesTrue(LauncherSubsystem.INSTANCE.Launch1());

        // AUTO LAUNCH 3 ARTIFACT
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(LauncherSubsystem.INSTANCE.Launch3());

        // MANUEL CONTROL OF FLIPPER
        Gamepads.gamepad1().circle().toggleOnBecomesTrue()
                .whenBecomesTrue(OuttakeFlipperSubsystem.INSTANCE.setFullOn)
                .whenBecomesFalse(OuttakeFlipperSubsystem.INSTANCE.setFullOff);
    }

    @Override
    public void onUpdate() {

        Follower.INSTANCE.update(Odometry.INSTANCE.getX(),Odometry.INSTANCE.getY(),Odometry.INSTANCE.getHeading());

        swerveDrivetrain.runDrive(Follower.INSTANCE.teleOpLinear(-gamepad2.left_stick_x,gamepad2.left_stick_y), new Vector2D(-gamepad2.right_stick_x,0));

//        swerveDrivetrain.simpleRunDrive(-gamepad2.left_stick_x,gamepad2.left_stick_y,-gamepad2.right_stick_x);

        telemetry.addData("FPS", timer.getTime()/ Math.pow(10.0,9));
//        telemetry.addLine(OuttakeWheelSubsystem.INSTANCE.debugString());
//        telemetry.addData("Rotary", RotarySubsystem.INSTANCE.debugText());
//        telemetry.addData("swerve Output: ", swerveDrivetrain.debugString());
        telemetry.addData("ODO Output: ", Odometry.INSTANCE.getPos() );
//        telemetry.addData("lerp timer: ", OuttakeWheelSubsystem.INSTANCE.lerp.time);
//        telemetry.addData("lerp oldTime: ", OuttakeWheelSubsystem.INSTANCE.lerp.oldTime);
        telemetry.addData("Flywheel withinrange: ", OuttakeWheelSubsystem.INSTANCE.withinRangeBool());
        telemetry.addData("Rotary withinrange: ", RotarySubsystem.INSTANCE.withinRangeBool());
        telemetry.update();
        timer.reset();

        //swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, movementScaler);
    }

}
