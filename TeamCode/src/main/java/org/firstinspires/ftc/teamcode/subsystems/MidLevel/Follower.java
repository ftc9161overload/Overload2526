package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.Vector2D;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Odometry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;

public class Follower implements Subsystem {

    public static final Follower INSTANCE = new Follower();


    private double xPos, yPos, heading;
    private double xTarget, yTarget, headingTarget;

    private boolean linearFollwer = false, headingFollower = false;

    private PDFLController xCon = new PDFLController(0.06,0,0,0.35), headingCon = new PDFLController(0.03,0,0,0.35);
    private double xErrorMin = 0.5, headingErrorMin = 0.1;


    public Command withinRangeLinear(double Range) {
        return new LambdaCommand("Follower within range?").setIsDone(() -> Math.abs(Math.hypot(xPos-xTarget,yPos-yTarget)) < Range);
    }
    public Command withinRangeHeading(double Range) {
        return new LambdaCommand("Follower within range?").setIsDone(() -> Math.abs(heading - headingTarget) < Range);
    }

    public void update(double xPos, double yPos, double heading) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;
    }

    public Command setLinear(double xTarget, double yTarget) {
        return new InstantCommand(() -> {
           this.xTarget =  xTarget;
           this.yTarget = yTarget;
        });
    }

    public Command setHeading(double headingTarget) {
        return new InstantCommand(() -> {
            this.headingTarget = headingTarget;
        });
    }

    public Command turnOnLinear = new InstantCommand(() -> {
        linearFollwer = true;
    });
    public Command turnOffLinear = new InstantCommand(() -> {
        linearFollwer = false;
    });
    public Command turnOnHeading = new InstantCommand(() -> {
        headingFollower = true;
    });
    public Command turnOffHeading = new InstantCommand(() -> {
        headingFollower = false;
    });



    public Vector2D getLinear() {
        xCon.update(Math.hypot(xPos-xTarget,yPos-yTarget));
        return new Vector2D(xCon.runPDFL(xErrorMin),0).rotate(Math.atan2(xTarget-xPos,yTarget-yPos));
    }

    public Vector2D getHeading() {
        headingCon.update(heading);
        return new Vector2D(headingCon.runPDFL(headingErrorMin),0);
    }

    public Vector2D teleOpLinear(double x, double y ) {
        return new Vector2D(x,y).rotate(-heading);
    }


}
