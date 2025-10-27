package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.SwervePodSubsystem;

/* Pedro Pathing Docs:  
https://pedropathing.com/docs/pathing/custom/drivetrain
*/

public class SwerveDrivetrain extends Drivetrain {

    private SwervePodSubsystem fr,fl,br,bl;

    public SwerveDrivetrain(HardwareMap hMap) {
        // swerve pod subsystems       X       Y    Servo  Motor  AnaInput 
        fr = new SwervePodSubsystem( 156.0,  156.0, "frs", "frm", "frsai", hMap); // Front Right
        fl = new SwervePodSubsystem(-156.0,  156.0, "fls", "flm", "flsai", hMap); // Front Left
        br = new SwervePodSubsystem( 156.0, -156.0, "brs", "brm", "brsai", hMap); // Back Right
        //bl = new SwervePodSubsystem(-156.0, -156.0, "bls", "blm", "blsai", hMap); // Back Left
    }
    
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        
        return new double[0];
    }

    @Override
    public void runDrive(double[] drivePowers) {

    }

    @Override
    public void updateConstants() {

    }

    @Override
    public void breakFollowing() {

    }

    @Override
    public void startTeleopDrive() {

    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {

    }

    @Override
    public double xVelocity() {
        return 0;
    }

    @Override
    public double yVelocity() {
        return 0;
    }

    @Override
    public void setXVelocity(double xMovement) {

    }

    @Override
    public void setYVelocity(double yMovement) {

    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public String debugString() {
        return "";
    }
}
