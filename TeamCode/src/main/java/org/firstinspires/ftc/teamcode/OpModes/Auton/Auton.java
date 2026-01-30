package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "CloseAuton", group = "Auton")
@Configurable
public class Auton extends NextFTCOpMode {
    public Auton() {
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


    private Command autonCommand = new SequentialGroup(
            Follower.INSTANCE.setLinear(30,0),
//            Follower.INSTANCE.withinRangeLinear(4),
//            Follower.INSTANCE.withinRangeHeading(.4),
            new Delay(2),
            LauncherSubsystem.INSTANCE.Launch3()
    );

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(IntakeSubsystem.INSTANCE, LauncherSubsystem.INSTANCE, OuttakeWheelSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        OuttakeWheelSubsystem.INSTANCE.targetSpeed = 0;
        swerveDrivetrain = new SwerveDrivetrain();

        Follower.INSTANCE.turnOnLinear.schedule();
        Follower.INSTANCE.turnOnHeading.schedule();
        Follower.INSTANCE.setHeading(0).schedule();
        Follower.INSTANCE.setLinear(0,0).schedule();
        Odometry.INSTANCE.reset.schedule();

        Odometry.INSTANCE.initReal();
        Odometry.INSTANCE.reset.schedule();
        RotarySubsystem.INSTANCE.reset();

        RotarySubsystem.INSTANCE.home.schedule();

        //RotarySubsystem.INSTANCE.resetOffset();
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        Follower.INSTANCE.turnOffHeading.schedule();
        Follower.INSTANCE.turnOffLinear.schedule();

        RotarySubsystem.INSTANCE.startRotary.schedule();
        RotarySubsystem.INSTANCE.locked = false;
        // INTAKE (HOLD TO USE)
        autonCommand.schedule();

    }

    @Override
    public void onUpdate() {

        Follower.INSTANCE.update(Odometry.INSTANCE.getX(),Odometry.INSTANCE.getY(),Odometry.INSTANCE.getHeading());


        swerveDrivetrain.runDrive(Follower.INSTANCE.getLinear(), Follower.INSTANCE.getHeading());

//        swerveDrivetrain.runDrive(Follower.INSTANCE.teleOpLinear(-gamepad2.left_stick_x,gamepad2.left_stick_y), new Vector2D(-gamepad2.right_stick_x,0));

//        swerveDrivetrain.simpleRunDrive(-gamepad2.left_stick_x,gamepad2.left_stick_y,-gamepad2.right_stick_x);

//        telemetry.addData("FPS", timer.getTime()/ Math.pow(10.0,9));
        telemetry.addLine(OuttakeWheelSubsystem.INSTANCE.debugString());
//        telemetry.addData("Rotary", RotarySubsystem.INSTANCE.debugText());
////        telemetry.addData("swerve Output: ", swerveDrivetrain.debugString());
        telemetry.addData("ODO Output: ", Odometry.INSTANCE.getPos() );
////        telemetry.addData("lerp timer: ", OuttakeWheelSubsystem.INSTANCE.lerp.time);
////        telemetry.addData("lerp oldTime: ", OuttakeWheelSubsystem.INSTANCE.lerp.oldTime);
        telemetry.addData("Flywheel withinrange: ", OuttakeWheelSubsystem.INSTANCE.withinRangeBool());
        telemetry.addData("Rotary withinrange: ", RotarySubsystem.INSTANCE.withinRangeBool());
        telemetry.update();
//        timer.reset();

        //swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, movementScaler);
    }

}
