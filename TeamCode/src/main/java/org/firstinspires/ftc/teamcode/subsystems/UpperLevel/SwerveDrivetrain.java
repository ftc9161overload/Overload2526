package org.firstinspires.ftc.teamcode.subsystems.UpperLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.SwervePodSubsystem;

import dev.nextftc.core.subsystems.Subsystem;

/* Pedro Pathing Docs:  
https://pedropathing.com/docs/pathing/custom/drivetrain
*/
@Configurable
public class SwerveDrivetrain implements Subsystem {

    public static int flOffset = -30+90;
    public static int frOffset = -35+90;
    public static int blOffset = 69-45;
    public static int brOffset = 255;

    //private constants SwerveDrivetrainConstants();
    private SwervePodSubsystem[] pods;

    private GoBildaPinpointDriver ppDriver;

    public SwerveDrivetrain(HardwareMap hMap) {
        SwervePodSubsystem fr = new SwervePodSubsystem( -156.0,  -156.0, UniConstants.DRIVE_FRONT_RIGHT_SERVO_STRING, UniConstants.DRIVE_FRONT_RIGHT_STRING, UniConstants.DRIVE_FRONT_RIGHT_ANALOG_INPUT, hMap); // Front Right
        SwervePodSubsystem fl = new SwervePodSubsystem(-156.0,  156.0, UniConstants.DRIVE_FRONT_LEFT_SERVO_STRING, UniConstants.DRIVE_FRONT_LEFT_STRING, UniConstants.DRIVE_FRONT_LEFT_ANALOG_INPUT, hMap); // Front Left
        SwervePodSubsystem br = new SwervePodSubsystem( 156.0, -156.0, UniConstants.DRIVE_BACK_RIGHT_SERVO_STRING, UniConstants.DRIVE_BACK_RIGHT_STRING,  UniConstants.DRIVE_BACK_RIGHT_ANALOG_INPUT, hMap); // Back Right
        SwervePodSubsystem bl = new SwervePodSubsystem(156.0, 156.0, UniConstants.DRIVE_BACK_LEFT_SERVO_STRING, UniConstants.DRIVE_BACK_LEFT_STRING, UniConstants.DRIVE_BACK_LEFT_ANALOG_INPUT, hMap); // Back Left


        bl.setServoReverse(true);
        bl.setServoMKII();


        fl.setServoOffsetDeg(flOffset);
        fr.setServoOffsetDeg(frOffset);
        bl.setServoOffsetDeg(blOffset);
        br.setServoOffsetDeg(brOffset);

        ppDriver = hMap.get(GoBildaPinpointDriver.class, "pinpoint");

        ppDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        ppDriver.setOffsets(-182,182, DistanceUnit.MM);
        ppDriver.resetPosAndIMU();


        pods = new SwervePodSubsystem[]{fl, fr, bl, br}; // Array of the pods so we can loop through in a for each and run functions on all of them :thumbs-up:
    }
    











    public void setServoPowZero() {
        for(SwervePodSubsystem pod : pods) {
            pod.setServoPower(0);
        }
    }
    public void setPosZero() {
        for(SwervePodSubsystem pod : pods) {
            pod.setPos(0);
        }
    }



    public void setMotorsToBrake() {
        for (SwervePodSubsystem pod : pods) {
            pod.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

    }

    public SwervePodSubsystem[] getSwervePods() {
        return pods;
    }

    public void simpleRunDrive(double x, double y, double rotation) {
        for (SwervePodSubsystem pod : pods) {
            pod.update(x, y, rotation);
        }

        ppDriver.update();
    }

    public void simpleRunDrive(double x, double y, double rotation, double movementScaler) {
        for (SwervePodSubsystem pod : pods) {
            pod.update(x, y, rotation, movementScaler);
        }
    }

    public void setMotorsToFloat() {
        for (SwervePodSubsystem pod : pods) {
            pod.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }
    }
    


    public void startTeleopDrive() {

    }


    public void startTeleopDrive(boolean brakeMode) {

    }




    public String debugString() {
        String returnStr = "";
        for (SwervePodSubsystem swerve : pods) {
            returnStr += swerve.debugText();
            returnStr += "\n\n";
        }

        returnStr += "X Pos: " + ppDriver.getPosX(DistanceUnit.INCH) + "\nY Pos: " + ppDriver.getPosY(DistanceUnit.INCH) + "\nHeading: " + ppDriver.getHeading(AngleUnit.DEGREES);

        return returnStr;
    }
}
