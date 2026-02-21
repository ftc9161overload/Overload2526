package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.Vector2D;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeFlipper;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Robot;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Rotary;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "TeleOp", group = "Gameday")
@Configurable
public class NextFTCTeleOp extends NextFTCOpMode {
    {
        addComponents(
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    JoinedTelemetry joinedTelemetry;

    private boolean slowmode = false;
    private double movementScaler = 1.0;
    public static double outtakePreset1 = 1900;
    public static double outtakePreset2 = 2560;


    @Override
    public void onInit() {


        addComponents(
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );


        Robot.INSTANCE.OpModeFullInit();


    }



    @Override
    public void onWaitForStart() {

        Robot.INSTANCE.waitForStart();

    }

    @Override
    public void onStartButtonPressed() {

        Robot.INSTANCE.TeleOpInit();

    }

    @Override
    public void onUpdate() {
    }

}
