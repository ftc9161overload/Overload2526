package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Rotary;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Robot;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.core.units.Distance;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "AutonClose2", group = "Auton2")
@Configurable
public class AutonClose2 extends NextFTCOpMode {
    JoinedTelemetry joinedTelemetry;
    public AutonClose2() {
        addComponents(
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    // THIS IS WHERE THE AUTON NEEDS TO BE WRITTEN
    private final Command autonCommand = new SequentialGroup(
//            Follower.INSTANCE.turnOnLinear,
//            Follower.INSTANCE.turnOnHeading,

            Follower.INSTANCE.set(new Pose2D(Distance.fromIn(0),Distance.fromIn(0),Angle.fromDeg(0))),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2)
//            Launcher.INSTANCE.Launch3(),
//            Launcher.INSTANCE.setHalfOn,

//            Follower.INSTANCE.setLinear(0, -19),
//            Follower.INSTANCE.setHeading(Math.toRadians(180)),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
//
//            Follower.INSTANCE.setLinear(-34, 0),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
////            Rotary.INSTANCE.rotateRotary,
//
//            Follower.INSTANCE.setLinear(34, 19),
//            Follower.INSTANCE.setHeading(Math.toRadians(140)),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
////            Launcher.INSTANCE.Launch3(),
////            Launcher.INSTANCE.setHalfOn,
//
//            Follower.INSTANCE.setLinear(0, -43),
//            Follower.INSTANCE.setHeading(Math.toRadians(180)),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
//
//            Follower.INSTANCE.setLinear(-34, 0),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
////            Rotary.INSTANCE.rotateRotary,
//
//            Follower.INSTANCE.setLinear(34, 43),
//            Follower.INSTANCE.setHeading(Math.toRadians(140)),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
////            Launcher.INSTANCE.Launch3(),
////            Launcher.INSTANCE.setHalfOn,
//
//            Follower.INSTANCE.setLinear(0, -67),
//            Follower.INSTANCE.setHeading(Math.toRadians(180)),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
//
//            Follower.INSTANCE.setLinear(-34, 0),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
////            Rotary.INSTANCE.rotateRotary,
//
//            Follower.INSTANCE.setLinear(34, 67),
//            Follower.INSTANCE.setHeading(Math.toRadians(180)),
//            Follower.INSTANCE.withinRangeLinear(0.5),
//            Follower.INSTANCE.withinRangeHeading(.2),
////            Launcher.INSTANCE.Launch3(),
//
//            Intake.INSTANCE.stop
    );

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Launcher.INSTANCE, OuttakeWheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        Robot.INSTANCE.OpModeFullInit();
        Follower.INSTANCE.turnOffLinear.schedule();
        Follower.INSTANCE.turnOffHeading.schedule();

        Follower.INSTANCE.setStartingPose(new Pose2D(Distance.fromIn(0),Distance.fromIn(0), Angle.fromDeg(0))).schedule();

    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {

        // INTAKE (HOLD TO USE)
        Robot.INSTANCE.setAuton();
        autonCommand.schedule();

    }

    // FOLLOWS THE AUTON SEQUENTIAL GROUP UP ABOVE
    @Override
    public void onUpdate() {

    }

}
