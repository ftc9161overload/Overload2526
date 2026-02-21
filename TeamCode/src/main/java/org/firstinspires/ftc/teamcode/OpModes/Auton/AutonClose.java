package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Rotary;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Robot;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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
            new SubsystemComponent(Robot.INSTANCE),
            BulkReadComponent.INSTANCE,
            BindingsComponent.INSTANCE
        );
    }





    // Should score in the blue goal
    private Command autonCommand = new SequentialGroup(
            OuttakeWheel.INSTANCE.setSpeed1,
            Follower.INSTANCE.setLinear(23,-22),
            Follower.INSTANCE.setHeading(Math.toRadians(140)),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,

            Follower.INSTANCE.setLinear(0, -19),
            Follower.INSTANCE.setHeading(Math.toRadians(180)),

            Follower.INSTANCE.setLinear(-34, 0),
            Rotary.INSTANCE.rotateRotary,

            Follower.INSTANCE.setLinear(34, 19),
            Follower.INSTANCE.setHeading(Math.toRadians(140)),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,

            Follower.INSTANCE.setLinear(0, -43),
            Follower.INSTANCE.setHeading(Math.toRadians(180)),

            Follower.INSTANCE.setLinear(-34, 0),
            Rotary.INSTANCE.rotateRotary,

            Follower.INSTANCE.setLinear(34, 43),
            Follower.INSTANCE.setHeading(Math.toRadians(140)),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,

            Follower.INSTANCE.setLinear(0, -67),
            Follower.INSTANCE.setHeading(Math.toRadians(180)),

            Follower.INSTANCE.setLinear(-34, 0),
            Rotary.INSTANCE.rotateRotary,

            Follower.INSTANCE.setLinear(34, 67),
            Follower.INSTANCE.setHeading(Math.toRadians(180)),
            Launcher.INSTANCE.Launch3(),

            Intake.INSTANCE.stop

    );

    @Override
    public void onInit() {

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Launcher.INSTANCE, OuttakeWheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        Robot.INSTANCE.setAuton();

        //RotarySubsystem.INSTANCE.resetOffset();
        Odometry.INSTANCE.setPos(22,125, Math.toRadians(144));
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {

        // INTAKE (HOLD TO USE)
        autonCommand.schedule();

    }

    @Override
    public void onUpdate() {


    }

}
