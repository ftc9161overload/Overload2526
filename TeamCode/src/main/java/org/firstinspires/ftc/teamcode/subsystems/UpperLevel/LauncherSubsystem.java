package org.firstinspires.ftc.teamcode.subsystems.UpperLevel;
import com.bylazar.configurables.annotations.Configurable;


import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeFlipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;

@Configurable
/**
 * LauncherSubsystem coordinates the rotary carousel, flywheel, and flipper
 * to launch game elements. Manages the complete launch sequence including
 * spin-up, chamber positioning, and element ejection.
 */
public class LauncherSubsystem extends SubsystemGroup {

    // ========== SINGLETON PATTERN ==========

    /** Singleton instance for easy access across OpModes */
    public static final LauncherSubsystem INSTANCE = new LauncherSubsystem();

    // ========== TIMING CONSTANTS ==========

    /** Time to wait for flipper to fully extend and launch element (seconds) */
    private static final double FLIP_DURATION = 0.7;

    /** Time to wait for flipper to retract before next action (seconds) */
    private static final double RETRACT_DURATION = 0.4;

    /** Delay between consecutive launches in multi-launch sequence (seconds) */
    private static final double LAUNCH_DELAY = 0.5;

    // ========== CONSTRUCTOR ==========

    /**
     * Private constructor enforces singleton pattern.
     * Registers all child subsystems with the SubsystemGroup.
     */
    private LauncherSubsystem() {
        super(
                OuttakeFlipperSubsystem.INSTANCE,  // Controls flipper mechanism
                OuttakeWheelSubsystem.INSTANCE,     // Controls flywheel speed
                RotarySubsystem.INSTANCE            // Controls chamber rotation
        );
    }

    // ========== HALF CHAMBER TOGGLE COMMANDS ==========

    /**
     * Disables half-chamber mode if currently enabled.
     * Returns flipper to safe position and aligns rotary to full chamber.
     * Used to transition from loading position to launch position.
     */
    public Command setHalfOff = new InstantCommand(() -> {
        // Only execute if currently in half-chamber mode
        if (RotarySubsystem.INSTANCE.halfChamber) {
            OuttakeFlipperSubsystem.INSTANCE.setFullOff.schedule();  // Retract flipper
            RotarySubsystem.INSTANCE.setHalfChamberOff.schedule();   // Align to chamber
        }
    });

    /**
     * Enables half-chamber mode if currently disabled.
     * Returns flipper to safe position and rotates to loading position.
     * Used to position rotary between chambers for easier element loading.
     */
    public Command setHalfOn = new InstantCommand(() -> {
        // Only execute if not already in half-chamber mode
        if (!RotarySubsystem.INSTANCE.halfChamber) {
            OuttakeFlipperSubsystem.INSTANCE.setFullOff.schedule();  // Retract flipper
            RotarySubsystem.INSTANCE.setHalfChamberOn.schedule();    // Move to half position
        }
    });

    // ========== LAUNCH SETUP ==========

    /**
     * Prepares the launcher for firing.
     * Spins up the flywheel to default speed if not already running.
     * Does not move rotary or flipper - only ensures flywheel is ready.
     * @return Command that starts flywheel spin-up
     */
    public Command setup() {
        return new InstantCommand(() -> {
            // Only spin up if flywheel isn't already running
            if (OuttakeWheelSubsystem.INSTANCE.targetSpeed == 0) {
                OuttakeWheelSubsystem.INSTANCE.setSpeed1.schedule();
            }
        });
    }

    // ========== SINGLE LAUNCH SEQUENCE ==========

    /**
     * Executes a complete single-element launch sequence.
     *
     * Sequence steps:
     * 1. Setup - Ensure flywheel is spinning
     * 2. Wait for flywheel to reach target speed
     * 3. Lock rotary to prevent chamber drift
     * 4. Wait for rotary to reach precise position
     * 5. Flip element into flywheel (0.7s)
     * 6. Retract flipper (0.4s)
     * 7. Unlock rotary for repositioning
     * 8. Advance to next chamber
     *
     * @return Sequential command group executing the launch
     */
    public Command Launch1() {
        return new SequentialGroup(
                setup(),                                      // Spin up flywheel if needed
                OuttakeWheelSubsystem.INSTANCE.withinRange(), // Wait for flywheel at speed
                RotarySubsystem.INSTANCE.lock,                // Lock chamber position
                RotarySubsystem.INSTANCE.withinRange(),       // Wait for precise alignment
                OuttakeFlipperSubsystem.INSTANCE.setFullOn,   // Flip element into wheel
                new Delay(FLIP_DURATION),                     // Wait for full extension
                OuttakeFlipperSubsystem.INSTANCE.setFullOff,  // Retract flipper
                new Delay(RETRACT_DURATION),                  // Wait for full retraction
                RotarySubsystem.INSTANCE.unlock,              // Allow rotary motion
                RotarySubsystem.INSTANCE.nextChamber          // Rotate to next chamber
        );
    }

    // ========== MULTI-LAUNCH SEQUENCE ==========

    /**
     * Launches three elements in sequence.
     * Executes Launch1() three times with brief delays between launches.
     * Total duration: ~6-7 seconds depending on flywheel spin-up time.
     *
     * @return Sequential command group launching all three elements
     */
    public Command Launch3() {
        return new SequentialGroup(
                Launch1(),                    // Launch from chamber 1
                new Delay(LAUNCH_DELAY),      // Brief pause for stability
                Launch1(),                    // Launch from chamber 2
                new Delay(LAUNCH_DELAY),      // Brief pause for stability
                Launch1()                     // Launch from chamber 3
        );
    }


    // Previous code
    // Sets the Outtake Wheel Subsystem to launch at the speed the method is named
    /*
    public void update() {
        switch(currentState) {
            case 0:
                start = false;
                chamber = 0;
                reset = false;


                break;
            case 1:
                timer.reset();
                stateUpdate(11);
                break;
            case 11:
                if (shootCount > 0) {
//                    stateUpdate(10);

                    outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_OFF);
                    if (timer.hasElapsedSeconds(1.2)) {
                        rotarySubsystem.nextChamber();
                        rotarySubsystem.setHalfChamber(false);
                        outtakeSubsystem.set(true);
                        stateUpdate(10);
                    }
                } else {
                    stateUpdate(0);
                }
                break;
            case 10:


                if(!reset) {
                    timer.reset();
                    reset = true;
                }
                rotarySubsystem.setHalfChamber(false);
//                if (rotarySubsystem.getHalfChamber()) {
//                    rotarySubsystem.setHalfChamber(false);
//                    if((getIsInPosition()) && (timer.hasElapsedSeconds(servoTime + 5))) {reset = false; stateUpdate(2);}
//                }
                if((getIsInPosition())) {reset = false; stateUpdate(2);}
                start = true;

                break;
            case 2:
                //outtakeSubsystem.setVel(outtakeTarget);
                outtakeSubsystem.set(true);
                if(!reset) {
                    timer.reset();
                    reset = true;
                }
                if((outtakeSubsystem.getVel() > outtakeSubsystem.getTargetVel() - 50) && (outtakeSubsystem.getVel() < outtakeSubsystem.getTargetVel() + 50)) {
                    if (timer.hasElapsedSeconds(0.5)) {
                        stateUpdate(3);
                        reset = false;
                    }
                }
                break;
            case 3:
                outtakeSubsystem.setServo(UniConstants.engagementLevel.ON);
                if(!reset) {
                    timer.reset();
                    reset = true;
                }
                if(timer.hasElapsedSeconds(servoTime)) {stateUpdate(4); reset = false;}

                break;

            case 4:
                outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_OFF);
                stateUpdate(5);
                break;
            case 5:
                if (shootCount > 0) {
                    stateUpdate(1);
                } else {
                    stateUpdate(0);
                }
                shootCount --;
                break;

        }

        debugText();
        outtakeSubsystem.periodic();
        rotarySubsystem.periodic();
    }
    public String debugText() {
        return "\nCurrent State: " + currentState + "\nlauncher chamber: " + chamber + "\ntime: " + timer.getTimeSeconds();
    }*/
}
