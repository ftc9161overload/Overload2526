package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;

public class SwerveDrivetrain extends Drivetrain {
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        return new double[0];
    }

    @Override
    public void updateConstants() {

    }

    @Override
    public void breakFollowing() {

    }

    @Override
    public void runDrive(double[] drivePowers) {

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
