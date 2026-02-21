package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

@Autonomous(name = "AutonTemplate", group = "Auton")
@Configurable
public class AutonTemplate extends NextFTCOpMode {
    JoinedTelemetry joinedTelemetry;
    public AutonTemplate() {
        addComponents(
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private static SwerveDrivetrain swerveDrivetrain;

    // THIS IS WHERE THE AUTON NEEDS TO BE WRITTEN
    private Command autonCommand = new SequentialGroup(
            Follower.INSTANCE.setLinear(-30,0),
            Follower.INSTANCE.withinRangeLinear(0.5),
            Follower.INSTANCE.withinRangeHeading(.2),
            Launcher.INSTANCE.Launch3()
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
        Odometry.INSTANCE.setPos(0,0, 0);
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
