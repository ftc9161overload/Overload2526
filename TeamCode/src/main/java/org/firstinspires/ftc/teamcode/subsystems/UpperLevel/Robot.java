package org.firstinspires.ftc.teamcode.subsystems.UpperLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.Vector2D;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeFlipper;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Rotary;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;

/**
 * RobotSubsystem - Main robot subsystem container
 * Groups all robot subsystems together using the singleton pattern.
 * This class serves as the central hub for all robot subsystems, manages
 * team color configuration, and provides centralized debug telemetry output.
 *
 * @Configurable annotation allows this class to be configured via FTC Dashboard
 */
@Configurable
public class Robot extends SubsystemGroup {


    // Singleton instance - ensures only one RobotSubsystem exists throughout the program
    public static final Robot INSTANCE = new Robot();

    /**
     * Private constructor - prevents external instantiation (singleton pattern)
     * Registers all subsystem instances with the parent SubsystemGroup for
     * coordinated lifecycle management (periodic updates, initialization, etc.)
     */
    private Robot() {
        super(Launcher.INSTANCE,
                Odometry.INSTANCE,
                Intake.INSTANCE,
//                VisionSubsystem.INSTANCE,
                Follower.INSTANCE);
    }

    // Swerve drivetrain subsystem - handles omnidirectional robot movement
    // Initialized first to ensure availability before INSTANCE creation
    public static SwerveDrivetrain swerveDrivetrain;

    // Telemetry object for displaying debug information to the driver station
    // Initialized in initialize() method when subsystem is set up
    JoinedTelemetry joinedTelemetry;
    Timer loopTimer = new Timer();

    private boolean teleSlowmode = false;
    private boolean teleOp = false;


    /**
     * Initialization method called when the subsystem is first created
     * Sets up the telemetry object for debug output
     * This is part of the subsystem lifecycle and is called automatically
     */
    @Override
    public void initialize() {
        swerveDrivetrain = new SwerveDrivetrain();
        // Assign telemetry to the singleton instance for use throughout the program
        joinedTelemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());

    }


    /**
     * Runs OpMode Initialization sequence for first run OpModes
     */

    public void OpModeFullInit() {

        OpModeSafeInit();
        Rotary.INSTANCE.home.schedule();

        Follower.INSTANCE.turnOffHeading.schedule();
        Follower.INSTANCE.turnOffLinear.schedule();

        Odometry.INSTANCE.reset.schedule();


    }

    /**
     * Runs OpMode Initialization for subsequent OpModes
     */

    public void OpModeSafeInit() {
        OuttakeWheel.INSTANCE.targetSpeed = 0;

        Gamepads.gamepad2().a().whenBecomesTrue(setTeamColorBlue);
        Gamepads.gamepad2().y().whenBecomesTrue(setTeamColorRed);
        Gamepads.gamepad1().a().whenBecomesTrue(setTeamColorBlue);
        Gamepads.gamepad1().y().whenBecomesTrue(setTeamColorRed);
    }

    /**
     * Sets Subsystems for teleOperation
     */

    public void TeleOpInit() {

        Follower.INSTANCE.turnOffLinear.schedule();
        Follower.INSTANCE.turnOffHeading.schedule();

        Rotary.INSTANCE.startOpMode.schedule();

        teleOp = true;



        BindingManager.reset();


        // INTAKE (HOLD TO USE)
        Gamepads.gamepad1().a()
                .whenBecomesTrue(Intake.INSTANCE.run)
                .whenBecomesFalse(Intake.INSTANCE.stop);



        // SET HALF ON THE ROTARY AND MAKES SURE FLIPPER IS NOT IN THE WAY
        Gamepads.gamepad1().y().toggleOnBecomesTrue()
                .whenBecomesTrue(Launcher.INSTANCE.setHalfOn)
                .whenBecomesFalse(Launcher.INSTANCE.setHalfOff);

        // OUTTAKE WHEEL STUFF
        Gamepads.gamepad1().x()
                .whenBecomesTrue(OuttakeWheel.INSTANCE.turnOff);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(OuttakeWheel.INSTANCE.setSpeedHigher);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(OuttakeWheel.INSTANCE.setSpeedLower);

        // CHANGE ROTARY POSITION
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Rotary.INSTANCE.nextChamber);

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Rotary.INSTANCE.previousChamber);

        // AUTO LAUNCH 1 ARTIFACT
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(Launcher.INSTANCE.Launch1());

        // AUTO LAUNCH 3 ARTIFACT
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(Launcher.INSTANCE.Launch3());

        // MANUEL CONTROL OF FLIPPER
        Gamepads.gamepad1().circle().toggleOnBecomesTrue()
                .whenBecomesTrue(OuttakeFlipper.INSTANCE.setFullOn)
                .whenBecomesFalse(OuttakeFlipper.INSTANCE.setFullOff);

        Gamepads.gamepad2().a()
                .whenBecomesTrue(Intake.INSTANCE.run)
                .whenBecomesFalse(Intake.INSTANCE.stop)
                .whenBecomesTrue(Rotary.INSTANCE.setHalfChamberOn)
                .whenBecomesFalse(Rotary.INSTANCE.setHalfChamberOff);


        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(Odometry.INSTANCE.reset)
                .whenBecomesTrue(Follower.INSTANCE.setStartingPose(0,0,0));

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(() -> teleSlowmode = true);

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(()-> teleSlowmode = false);

        Gamepads.gamepad2().x().whenBecomesTrue(Launcher.INSTANCE.launchGreen);
        Gamepads.gamepad2().b().whenBecomesTrue(Launcher.INSTANCE.launchPurple);
        Gamepads.gamepad2().y().whenBecomesTrue(Launcher.INSTANCE.Launch3());

        Gamepads.gamepad2().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(OuttakeWheel.INSTANCE.setSpeedHigher);

        Gamepads.gamepad2().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(OuttakeWheel.INSTANCE.setSpeedLower);

        Gamepads.gamepad2().dpadUp().toggleOnBecomesTrue().
                whenBecomesTrue(Follower.INSTANCE.turnOffFieldCentric)
                .whenBecomesFalse(Follower.INSTANCE.turnOnFieldCentric);

        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(Rotary.INSTANCE.nextChamber);
        Gamepads.gamepad2().dpadRight().whenBecomesTrue(Rotary.INSTANCE.previousChamber);


    }



    public void waitForStart() {

        joinedTelemetry.addData("Team Color Select", "Press A (Or Bottom Button) to select BLUE\nPress Y (Or Top Button) to select RED");
        joinedTelemetry.addData("Current Team Color", Follower.INSTANCE.teamcolor.toString());

//        if(ActiveOpMode.gamepad2().a || ActiveOpMode.gamepad1().a) {
//            setTeamColorBlue.schedule();
//        }
//        if (ActiveOpMode.gamepad2().y || ActiveOpMode.gamepad1().y) {
//            setTeamColorRed.schedule();
//        }

        loopTimer.reset();
    }


    public void onStart() {

    }

    /**
     * TEAMCOLOR enum - Represents which alliance the robot is on during a match
     * Used to adjust autonomous routines, field-relative movements, and scoring positions
     */
    public enum TEAMCOLOR {
        RED,   // Red alliance - robot on red side
        BLUE   // Blue alliance - robot on blue side
    }

    // Current team color - defaults to BLUE alliance
    // This should be set before autonomous or teleop based on alliance station
    public static TEAMCOLOR teamColor = TEAMCOLOR.BLUE;


    public void setAuton() {
        teleOp = false;
        OuttakeWheel.INSTANCE.targetSpeed = 0;
        swerveDrivetrain = new SwerveDrivetrain();

        Follower.INSTANCE.turnOnLinear.schedule();
        Follower.INSTANCE.turnOnHeading.schedule();
        Follower.INSTANCE.setHeading(0).schedule();
        Follower.INSTANCE.setLinear(0,0).schedule();

//        Odometry.INSTANCE.initReal();
//        Odometry.INSTANCE.reset.schedule();
//        RotarySubsystem.INSTANCE.reset();

    }



    /**
     * Sets the team color for the robot and propagates it to dependent subsystems
     * This method should be called during initialization based on the alliance station
     *
     * @param color String representing the team color ("blue" or "Blue" for BLUE, anything else for RED)
     */
    public void setTeamColor(TEAMCOLOR color) {
        // Check if the color is blue (case-insensitive comparison)
        teamColor = color;
        Follower.INSTANCE.teamcolor = color;
    }

    public InstantCommand setTeamColorBlue = new InstantCommand(() -> Follower.INSTANCE.teamcolor = TEAMCOLOR.BLUE);
    public InstantCommand setTeamColorRed = new InstantCommand(() -> Follower.INSTANCE.teamcolor = TEAMCOLOR.RED);

    // Flag to enable/disable telemetry debug output - defaults to disabled
    // When true, periodic() method will output diagnostic information
    // These flags are configured via FTC Dashboard using the @Configurable annotation
    public static boolean isTelemetry = false;
    public static boolean OdoTelemetry = false;
    public static boolean loopTimeTelemetry = true;
    public static boolean rotaryTelemetry = false;
    public static boolean outtakeTelemetry = false;
    public static boolean drivetrainTelemetry = false;
    public static boolean intakeTelemetry = false;
    public static boolean colorTelemetry = false;


    /**
     * Periodic method called continuously during OpMode execution
     * Handles debug telemetry output when enabled
     * This is part of the subsystem lifecycle and runs every loop iteration
     */
    @Override
    public void periodic() {

        if (teleOp) {
//            swerveDrivetrain.simpleRunDrive(
//                    -ActiveOpMode.gamepad2().left_stick_x * (teleSlowmode ? 0.5 : 1),
//                    ActiveOpMode.gamepad2().left_stick_y  * (teleSlowmode ? 0.5 : 1),
//                    ActiveOpMode.gamepad2().right_stick_x  * (teleSlowmode ? 0.5 : 1)
//            );

            swerveDrivetrain.runDrive(Follower.INSTANCE.getTeleOpLinear(
                    -ActiveOpMode.gamepad2().left_stick_x * (teleSlowmode ? 0.5 : 1),
                    ActiveOpMode.gamepad2().left_stick_y  * (teleSlowmode ? 0.5 : 1)
            ),
                    new Vector2D(-ActiveOpMode.gamepad2().right_stick_x  * (teleSlowmode ? 0.5 : 1),0)
            );
        } else {
            Follower.INSTANCE.update(Odometry.INSTANCE.getX(),Odometry.INSTANCE.getY(),Odometry.INSTANCE.getHeading());
            swerveDrivetrain.runDrive(Follower.INSTANCE.getLinear(), Follower.INSTANCE.getHeading());
        }
        Follower.INSTANCE.update(Odometry.INSTANCE.getX(),Odometry.INSTANCE.getY(),Odometry.INSTANCE.getHeading());



        if (OdoTelemetry) {
            joinedTelemetry.addLine(Odometry.INSTANCE.debugText());
        }

        // Only output debug info if telemetry is enabled
        if(isTelemetry) {
            // Enable vision subsystem debug visualization
            //VisionSubsystem.INSTANCE.setDebugMode(true);
            // Display follower subsystem debug information to driver station
            joinedTelemetry.addLine(Follower.INSTANCE.debugText());
        }

        if (loopTimeTelemetry) {
            joinedTelemetry.addData("Loop Time: ", loopTimer.getTime()/ Math.pow(10.0,9));
        }

//         Display rotary subsystem telemetry if enabled
        if(rotaryTelemetry) {
            joinedTelemetry.addLine(Rotary.INSTANCE.debugText());
        }

        if(colorTelemetry) {
            joinedTelemetry.addLine(Rotary.INSTANCE.debugColors());
        }

        // Display outtake subsystem telemetry if enabled
        if(outtakeTelemetry) {
            joinedTelemetry.addLine(OuttakeFlipper.INSTANCE.debugText());
            joinedTelemetry.addLine(OuttakeWheel.INSTANCE.debugText());
        }

        // Display drivetrain subsystem telemetry if enabled
        if(drivetrainTelemetry) {
            joinedTelemetry.addLine(swerveDrivetrain.debugText());
        }

        // Display intake subsystem telemetry if enabled
        if(intakeTelemetry) {
       //     joinedTelemetry.addLine(IntakeSubsystem.INSTANCE.debugText());
        }
        loopTimer.reset();
        joinedTelemetry.update();
    }
}