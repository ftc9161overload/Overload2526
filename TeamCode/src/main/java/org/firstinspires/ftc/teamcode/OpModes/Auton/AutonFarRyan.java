package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Rotary;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Robot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "AutonFarRyan", group = "AutonFarRyan")
@Configurable
public class AutonFarRyan extends NextFTCOpMode {
    JoinedTelemetry joinedTelemetry;
    public AutonFarRyan() {
        addComponents(
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    // THIS IS WHERE THE AUTON NEEDS TO BE WRITTEN
    private final Command autonCommand = new SequentialGroup(
            Follower.INSTANCE.turnOnLinear,
            Follower.INSTANCE.turnOnHeading,

            Follower.INSTANCE.setLinear(Poses.farGoal),
            Follower.INSTANCE.turnToGoal(),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,


            Follower.INSTANCE.set(Poses.spike3Start),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Intake.INSTANCE.run,

            Follower.INSTANCE.set(Poses.spike3End),
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Intake.INSTANCE.stop,

            Follower.INSTANCE.set(Poses.farGoal),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,

            Follower.INSTANCE.set(Poses.spike2Start),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Intake.INSTANCE.run,

            Follower.INSTANCE.set(Poses.spike2End),
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Intake.INSTANCE.stop,

            Follower.INSTANCE.set(Poses.farGoal),
            Follower.INSTANCE.turnToGoal(),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,

            Follower.INSTANCE.set(Poses.spike1Start),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Intake.INSTANCE.run,

            Follower.INSTANCE.set(Poses.spike1End),
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            new Delay(.5),
            Rotary.INSTANCE.nextChamber,
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Intake.INSTANCE.stop,

            Follower.INSTANCE.set(Poses.farGoal),
            Follower.INSTANCE.turnToGoal(),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,



            Intake.INSTANCE.stop
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

        Follower.INSTANCE.setStartingPose(Poses.startFar).schedule();

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
