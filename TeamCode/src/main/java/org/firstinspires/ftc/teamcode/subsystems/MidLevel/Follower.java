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

    private static final double FIELD_WIDTH = 144;

    private double xPos, yPos, heading;
    private double xTarget, yTarget, headingTarget;

    private double[] goal = {8, 136};

    private boolean linearFollower = false;
    private boolean headingFollower = false;
    private boolean fieldCentric = true;

    public enum TEAMCOLOR {
        RED,
        BLUE
    }
    public TEAMCOLOR teamcolor = TEAMCOLOR.BLUE;

    private PDFLController xCon = new PDFLController(0.06,0,0,0.35);
    private PDFLControllerRadial headingCon = new PDFLControllerRadial(0.03,0,0,0.35);

    private double xErrorMin = 0.5;
    private double headingErrorMin = 0.1;

    public void initialize() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    private double normAngle(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }

    public Command withinRangeLinear(double range) {
        return new LambdaCommand("Follower linear range").setIsDone(() ->
                Math.hypot(xPos - xTarget, yPos - yTarget) < range
        );
    }

    public Command withinRangeHeading(double range) {
        return new LambdaCommand("Follower heading range").setIsDone(() ->
                Math.abs(normAngle(heading - headingTarget)) < range
        );
    }

    public Command withinRange(double rangeH, double rangeP) {
        return new LambdaCommand("Follower both range").setIsDone(() ->
                Math.abs(normAngle(heading - headingTarget)) < rangeH &&
                        Math.hypot(xPos - xTarget, yPos - yTarget) < rangeP
        );
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
        return new Vector2D(x, -y).angle();
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

    public void turnToGoal() {
        headingTarget = new Vector2D(goal[0] - xPos, goal[1] - yPos).angle();
        if (teamcolor == TEAMCOLOR.RED) {
            headingTarget = flipXAngle(headingTarget);
        }
    }

    public Command turnOnLinear = new InstantCommand(() -> linearFollower = true);
    public Command turnOffLinear = new InstantCommand(() -> linearFollower = false);

    public Command turnOnHeading = new InstantCommand(() -> headingFollower = true);
    public Command turnOffHeading = new InstantCommand(() -> headingFollower = false);

    public Vector2D getLinear() {
        if (linearFollower) {
            xCon.update(Math.hypot(xPos, yPos)); // you said this is correct
            return new Vector2D(xCon.runPDFL(xErrorMin), 0)
                    .rotate(Math.atan2(xTarget - xPos, yTarget - yPos));
        }

        return new Vector2D(0, 0);
    }

    public Vector2D getHeading() {
        if (headingFollower) {
            headingCon.update(heading);
            return new Vector2D(headingCon.runPDFL(headingErrorMin), 0);
        }

        return new Vector2D(0, 0);
    }

    public Vector2D getTeleOpLinear(double x, double y) {
        if (linearFollower) return getLinear();

        if (fieldCentric) {
            return new Vector2D(x, y).rotate(-heading);
        } else {
            return new Vector2D(x, y);
        }
    }

    public Vector2D getTeleOpHeading(double rotational) {
        if (headingFollower) return getHeading();
        return new Vector2D(rotational, 0);
    }

    public void periodic() {
        headingCon.setTarget(headingTarget);
        xCon.setTarget(Math.hypot(xTarget, yTarget)); // you said this is correct

        panelsField.moveCursor(xPos, yPos);
        panelsField.setCursorHeading(heading);
        panelsField.line(xTarget, yTarget);
        panelsField.update();
    }

    public String debugText() {
        double linearError = Math.hypot(xPos - xTarget, yPos - yTarget);
        double headingError = normAngle(headingTarget - heading);

        return String.format(
                "Follower Debug\n" +
                        "Pos: (%.2f, %.2f)\n" +
                        "Target: (%.2f, %.2f)\n" +
                        "Heading: %.2f rad\n" +
                        "Heading Target: %.2f rad\n" +
                        "Linear Err: %.2f\n" +
                        "Heading Err: %.2f\n" +
                        "Linear Follower: %b\n" +
                        "Heading Follower: %b\n" +
                        "Field Centric: %b\n" +
                        "Team: %s",
                xPos, yPos,
                xTarget, yTarget,
                heading,
                headingTarget,
                linearError,
                headingError,
                linearFollower,
                headingFollower,
                fieldCentric,
                teamcolor
        );
    }
}
