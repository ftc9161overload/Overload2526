package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
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
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "CloseAuton2", group = "Auton")
@Configurable
public class Auton2 extends NextFTCOpMode {
    public Auton2() {
        addComponents(
            new SubsystemComponent(Robot.INSTANCE),
            BulkReadComponent.INSTANCE,
            BindingsComponent.INSTANCE
        );
    }



    private Command autonCommand = new SequentialGroup(
            Follower.INSTANCE.setLinear(20,20),
            new Delay(4),
            Follower.INSTANCE.setLinear(20,20),
            new Delay(4),
            Follower.INSTANCE.setLinear(20,20),
            new Delay(2),

            Follower.INSTANCE.setHeading(3/4 * Math.PI)
,//            Follower.INSTANCE.withinRangeLinear(4),
//            Follower.INSTANCE.withinRangeHeading(.4),
            new Delay(2),
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

        //RotarySubsystem.INSTANCE.resetOffset();
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {


    }

    @Override
    public void onUpdate() {

    }

}
