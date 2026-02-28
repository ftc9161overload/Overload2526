package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Pose2D;
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

@Autonomous(name = "AutonTest", group = "Auton2")
@Configurable
public class AutonTest2 extends NextFTCOpMode {
    JoinedTelemetry joinedTelemetry;
    public AutonTest2() {
        addComponents(
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static double targetX = 0, targetY = 0, targetHeading = 0;


    // THIS IS WHERE THE AUTON NEEDS TO BE WRITTEN
    private final Command autonCommand = new SequentialGroup(
//            Follower.INSTANCE.turnOnLinear,
//            Follower.INSTANCE.turnOnHeading,

            Follower.INSTANCE.set(new Pose2D(targetX,targetY,targetHeading))
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

        Follower.INSTANCE.setStartingPose(new Pose2D(0,0,Math.toRadians(0))).schedule();

    }

    @Override
    public void onWaitForStart() {
        Robot.INSTANCE.waitForStart();

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
        Follower.INSTANCE.set(new Pose2D(targetX,targetY,Math.toRadians(targetHeading))).schedule();

    }

}
