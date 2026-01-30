package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Close Auton", group = "Auton")
@Configurable
public class AutonClose extends NextFTCOpMode {
    JoinedTelemetry joinedTelemetry;
    public AutonClose() {
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

    // Time Delay
    private Command time = new InstantCommand(() -> {
        timer.reset();
        boolean b = timer.hasElapsedSeconds(0.5);
    });

    // Rotates the rotary to collect balls
    private Command rotate = new SequentialGroup(
            time,
            RotarySubsystem.INSTANCE.nextChamber,
            time,
            RotarySubsystem.INSTANCE.nextChamber
    );

    // Should score in the blue goal
    private Command autonCommand = new SequentialGroup(
            OuttakeWheelSubsystem.INSTANCE.setSpeed1,
            Follower.INSTANCE.setLinear(23,-22),
            Follower.INSTANCE.setHeading(Math.toRadians(140)),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            LauncherSubsystem.INSTANCE.Launch3(),
            LauncherSubsystem.INSTANCE.setHalfOn,

            Follower.INSTANCE.setLinear(0, -19),
            Follower.INSTANCE.setHeading(Math.toRadians(180)),

            Follower.INSTANCE.setLinear(-34, 0),
            rotate,

            Follower.INSTANCE.setLinear(34, 19),
            Follower.INSTANCE.setHeading(Math.toRadians(140)),
            LauncherSubsystem.INSTANCE.Launch3(),
            LauncherSubsystem.INSTANCE.setHalfOn,

            Follower.INSTANCE.setLinear(0, -43),
            Follower.INSTANCE.setHeading(Math.toRadians(180)),

            Follower.INSTANCE.setLinear(-34, 0),
            rotate,

            Follower.INSTANCE.setLinear(34, 43),
            Follower.INSTANCE.setHeading(Math.toRadians(140)),
            LauncherSubsystem.INSTANCE.Launch3(),
            LauncherSubsystem.INSTANCE.setHalfOn,

            Follower.INSTANCE.setLinear(0, -67),
            Follower.INSTANCE.setHeading(Math.toRadians(180)),

            Follower.INSTANCE.setLinear(-34, 0),
            rotate,

            Follower.INSTANCE.setLinear(34, 67),
            Follower.INSTANCE.setHeading(Math.toRadians(180)),
            LauncherSubsystem.INSTANCE.Launch3(),

            IntakeSubsystem.INSTANCE.stop

    );

    @Override
    public void onInit() {

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
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
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        //RotarySubsystem.INSTANCE.resetOffset();
        Odometry.INSTANCE.setPos(22,125, Math.toRadians(144));
    }

    @Override
    public void onWaitForStart() {
        if (gamepad2.a) {
            Follower.INSTANCE.teamcolor = Follower.TEAMCOLOR.BLUE;
        } else if (gamepad2.y) {
            Follower.INSTANCE.teamcolor = Follower.TEAMCOLOR.RED;
        }
        joinedTelemetry.addData("Team Color Select", "Press A (Or Bottom Button) to select BLUE\nPress Y (Or Top Button) to select RED");
        joinedTelemetry.addData("Current Team Color", Follower.INSTANCE.teamcolor.toString());
        joinedTelemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
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
//        telemetry.addLine(OuttakeWheelSubsystem.INSTANCE.debugString());
//        telemetry.addData("Rotary", RotarySubsystem.INSTANCE.debugText());
////        telemetry.addData("swerve Output: ", swerveDrivetrain.debugString());
//        telemetry.addData("ODO Output: ", Odometry.INSTANCE.getPos() );
////        telemetry.addData("lerp timer: ", OuttakeWheelSubsystem.INSTANCE.lerp.time);
////        telemetry.addData("lerp oldTime: ", OuttakeWheelSubsystem.INSTANCE.lerp.oldTime);
//        telemetry.addData("Flywheel withinrange: ", OuttakeWheelSubsystem.INSTANCE.withinRangeBool());
//        telemetry.addData("Rotary withinrange: ", RotarySubsystem.INSTANCE.withinRangeBool());
        telemetry.addData("Follower", Follower.INSTANCE.debugText());
        telemetry.update();
//        timer.reset();

        //swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, movementScaler);
    }

}
