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
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }



    // THIS IS WHERE THE AUTON NEEDS TO BE WRITTEN
    private Command autonCommand = new SequentialGroup(
            //OuttakeWheelSubsystem.INSTANCE.setSpeed3,
            Follower.INSTANCE.set(64,15, Math.toRadians(75)),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,
            Intake.INSTANCE.run,

            Follower.INSTANCE.set(64, 36, Math.toRadians(0)),
            Follower.INSTANCE.setLinear(104, 36),
            Rotary.INSTANCE.rotateRotary,
            Follower.INSTANCE.set(64, 15, Math.toRadians(75)),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,

            Follower.INSTANCE.set(64, 60, Math.toRadians(0)),
            Follower.INSTANCE.setLinear(104, 60),
            Rotary.INSTANCE.rotateRotary,
            Follower.INSTANCE.set(64, 15, Math.toRadians(75)),
            Launcher.INSTANCE.Launch3(),
            Launcher.INSTANCE.setHalfOn,

            Follower.INSTANCE.set(64, 84, Math.toRadians(0)),
            Follower.INSTANCE.setLinear(104, 84),
            Rotary.INSTANCE.rotateRotary,
            Follower.INSTANCE.set(64, 15, Math.toRadians(75)),
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
                Robot.INSTANCE.setAuton();

        // DON'T FORGET TO CHANGE THIS SO THE ROBOT KNOWS WHERE IT IS AT!!!
        Odometry.INSTANCE.setPos(64,8, Math.toRadians(90));
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {

        // INTAKE (HOLD TO USE)
        autonCommand.schedule();

    }

    // FOLLOWS THE AUTON SEQUENTIAL GROUP UP ABOVE
    @Override
    public void onUpdate() {

    }

}
