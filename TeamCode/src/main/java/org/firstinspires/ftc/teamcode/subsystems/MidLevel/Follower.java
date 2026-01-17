package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.Vector2D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;

public class Follower implements Subsystem {

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    public static final Follower INSTANCE = new Follower();

    private static final double FIELD_WIDTH = 144; // Adjust to actual field width

    private double xPos, yPos, heading;
    private double xTarget, yTarget, headingTarget;
    private double[] goal = {8, 136};
    private boolean linearFollwer = false, headingFollower = false;

    private boolean fieldCentric = true;

    public enum TEAMCOLOR {
        RED,
        BLUE;

        @NonNull
        public String toString(){
            if (this == RED){
                return "RED";
            } else {
                return "BLUE";
            }
        }
    }
    public TEAMCOLOR teamcolor = TEAMCOLOR.BLUE;

    private PDFLController xCon = new PDFLController(0.06,0,0,0.35);
    private PDFLControllerRadial headingCon = new PDFLControllerRadial(0.03,0,0,0.35);
    private double xErrorMin = 0.5, headingErrorMin = 0.1;

    public void initialize(){
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public Command withinRangeLinear(double Range) {
        return new LambdaCommand("Follower within range?").setIsDone(() -> Math.abs(Math.hypot(xPos-xTarget,yPos-yTarget)) < Range);
    }
    public Command withinRangeHeading(double Range) {
        return new LambdaCommand("Follower within range?").setIsDone(() -> Math.abs(heading - headingTarget) < Range);
    }

    public Command withinRange(double RangeH, double RangeP) {
        return new LambdaCommand("Follower within range?").setIsDone(() -> ((Math.abs(heading - headingTarget) < RangeH) && (Math.abs(Math.hypot(xPos-xTarget,yPos-yTarget)) < RangeP)) );
    }

    public void update(double xPos, double yPos, double heading) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;
    }

    private double flipX(double x) {
        return FIELD_WIDTH - x;
    }
    private double flipXAngle(double angle) {
        double x = Math.cos(angle);
        double y = Math.sin(angle);
        return new Vector2D(-x, y).angle();
    }

    public Command turnOnFieldCentric = new InstantCommand(() -> fieldCentric = true);
    public Command turnOffFieldCentric = new InstantCommand(() -> fieldCentric = false);

    /**
     * Sets target coordinates and heading, flipping x and heading for RED team.
     */
    public Command set(double xTarget, double yTarget, double headingTarget) {
        return new InstantCommand(() -> {
            if (teamcolor == TEAMCOLOR.RED) {
                this.xTarget = flipX(xTarget);
                this.yTarget = yTarget;
                this.headingTarget = flipXAngle(headingTarget);
            } else {
                this.xTarget = xTarget;
                this.yTarget = yTarget;
                this.headingTarget = headingTarget;
            }
        });
    }

    public Command setLinear(double xTarget, double yTarget) {
        return new InstantCommand(() -> {
            if (teamcolor == TEAMCOLOR.RED) {
                this.xTarget = flipX(xTarget);
                this.yTarget = yTarget;
            } else {
                this.xTarget = xTarget;
                this.yTarget = yTarget;
            }
        });
    }

    public Command setHeading(double headingTarget) {
        return new InstantCommand(() -> {
            if (teamcolor == TEAMCOLOR.RED) {
                this.headingTarget = flipXAngle(headingTarget);
            } else {
                this.headingTarget = headingTarget;
            }
        });
    }

    public void turnToGoal(){
        headingTarget = new Vector2D((goal[0] - xPos),(goal[1] - yPos)).angle();
        if (teamcolor == TEAMCOLOR.RED) {
            headingTarget = flipXAngle(headingTarget);
        }
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

    public Vector2D getTeleOpLinear(double x, double y ) {
        if(linearFollwer) {
            xCon.update(Math.hypot(xPos-xTarget,yPos-yTarget));
            return new Vector2D(xCon.runPDFL(xErrorMin),0).rotate(Math.atan2(xTarget-xPos,yTarget-yPos));
        } else {
            if (fieldCentric) {
                return new Vector2D(x, y).rotate(-heading);
            } else {
                return new Vector2D(x, y);
            }
        }
    }

    public Vector2D getTeleOpHeading(double rotational) {
        if(headingFollower) {
            headingCon.update(heading);
            return new Vector2D(headingCon.runPDFL(headingErrorMin),0);
        } else {

            return new Vector2D(rotational, 0);
        }
    }

    public void periodic(){
        panelsField.moveCursor(xPos, yPos);
        panelsField.setCursorHeading(heading);
        panelsField.line(xTarget, yTarget);
        panelsField.update();
    }
}
