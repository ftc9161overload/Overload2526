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
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.commands.utility.InstantCommand;

@Autonomous(name = "Far Auton", group = "Auton")
@Configurable
public class AutonFar extends NextFTCOpMode {
    JoinedTelemetry joinedTelemetry;
    public AutonFar() {
        addComponents(
                new SubsystemComponent(IntakeSubsystem.INSTANCE, LauncherSubsystem.INSTANCE, OuttakeWheelSubsystem.INSTANCE,Odometry.INSTANCE, Follower.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private static SwerveDrivetrain swerveDrivetrain;
    private Timer timer = new Timer();


    private Command time = new InstantCommand(() -> {
        timer.reset();
        boolean b = timer.hasElapsedSeconds(0.5);
    });

    // Time Delay
    private Command rotate = new SequentialGroup(
            time,
            RotarySubsystem.INSTANCE.nextChamber,
            time,
            RotarySubsystem.INSTANCE.nextChamber
    );

    // THIS IS WHERE THE AUTON NEEDS TO BE WRITTEN
    private Command autonCommand = new SequentialGroup(
            //OuttakeWheelSubsystem.INSTANCE.setSpeed3,
            Follower.INSTANCE.set(64,15, Math.toRadians(75)),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            LauncherSubsystem.INSTANCE.Launch3(),
            LauncherSubsystem.INSTANCE.setHalfOn,
            IntakeSubsystem.INSTANCE.run,

            Follower.INSTANCE.set(64, 36, Math.toRadians(0)),
            Follower.INSTANCE.setLinear(104, 36),
            rotate,
            Follower.INSTANCE.set(64, 15, Math.toRadians(75)),
            LauncherSubsystem.INSTANCE.Launch3(),
            LauncherSubsystem.INSTANCE.setHalfOn,

            Follower.INSTANCE.set(64, 60, Math.toRadians(0)),
            Follower.INSTANCE.setLinear(104, 60),
            rotate,
            Follower.INSTANCE.set(64, 15, Math.toRadians(75)),
            LauncherSubsystem.INSTANCE.Launch3(),
            LauncherSubsystem.INSTANCE.setHalfOn,

            Follower.INSTANCE.set(64, 84, Math.toRadians(0)),
            Follower.INSTANCE.setLinear(104, 84),
            rotate,
            Follower.INSTANCE.set(64, 15, Math.toRadians(75)),
            LauncherSubsystem.INSTANCE.Launch3(),
            LauncherSubsystem.INSTANCE.setHalfOn,
            IntakeSubsystem.INSTANCE.stop

    );

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(IntakeSubsystem.INSTANCE, LauncherSubsystem.INSTANCE, OuttakeWheelSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        OuttakeWheelSubsystem.INSTANCE.targetSpeed = 0;
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);

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

        // DON'T FORGET TO CHANGE THIS SO THE ROBOT KNOWS WHERE IT IS AT!!!
        Odometry.INSTANCE.setPos(64,8, Math.toRadians(90));
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

    // FOLLOWS THE AUTON SEQUENTIAL GROUP UP ABOVE
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
