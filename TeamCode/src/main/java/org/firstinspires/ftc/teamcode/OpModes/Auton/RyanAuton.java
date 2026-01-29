package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Ryan's Auton", group = "Auton")
@Configurable
public class RyanAuton extends NextFTCOpMode {
    /*
    //public static double outtakePower = 10;
    public static double servoPos = 0.3; // 0.3 off, 0.8 on
    private double movementScaler = 1.0;
    public static double outtakePreset1 = 1900;
    public static double outtakePreset2 = 2560;

    private String userInterface = "";
    private boolean thingy = true;

    private static LauncherSubsystem launcherSubsystem;
    //private static RotarySubsystem rotarySubsystem;
    private static IntakeSubsystem intakeSubsystem;
    //private static OuttakeSubsystem outtakeSubsystem;
    private static SwerveDrivetrain swerveDrivetrain;

    private Timer timer = new Timer();
    private boolean setShot = false;
    @Override
    public void onInit() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap);
        launcherSubsystem.rotarySubsystem = new RotarySubsystem(hardwareMap, UniConstants.ROTARY_MOTOR_STRING);
        //intakeSubsystem = new IntakeSubsystem(UniConstants.INTAKE_MOTOR_STRING, hardwareMap);
        launcherSubsystem.outtakeSubsystem = new OuttakeSubsystem();
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);


        launcherSubsystem.rotarySubsystem.setIsOn(true);
    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        timer.reset();

    }

    @Override
    public void onUpdate() {

        launcherSubsystem.outtakeSubsystem.setVel(1900);
        launcherSubsystem.outtakeSubsystem.set(true);
        if (thingy) {
            launcherSubsystem.rotarySubsystem.nextChamber();
            launcherSubsystem.rotarySubsystem.nextChamber();
            thingy = false;
        }
        if (timer.hasElapsedSeconds(1) && !timer.hasElapsedSeconds(3.5)){
            swerveDrivetrain.simpleRunDrive(0,-0.5,0);
        } else if (timer.hasElapsedSeconds(4) && !setShot) {
            launcherSubsystem.setShootCount(5);
            launcherSubsystem.stateUpdate(1);
            setShot = true;
        }
        else if (timer.hasElapsedSeconds(11) && !timer.hasElapsed(12)) {
            swerveDrivetrain.simpleRunDrive(0.5,0,0);

        } else {
            swerveDrivetrain.simpleRunDrive(0,0,0);
        }

        intakeSubsystem.periodic();
        launcherSubsystem.update();
    }
*/
}
