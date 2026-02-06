package org.firstinspires.ftc.teamcode.subsystems.LowLevel_General;

import dev.nextftc.core.units.Angle;
import dev.nextftc.core.units.Distance;

public class Pose2D {
    public final Distance x;
    public final Distance y;
    public final Angle heading;

    public Pose2D(Distance x, Distance y, Angle heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Convenience constructors
    public Pose2D(double xIn, double yIn, double headingRad) {
        this(Distance.fromIn(xIn), Distance.fromIn(yIn), Angle.fromRad(headingRad));
    }

    // Just position, no heading
    public Pose2D(Distance x, Distance y) {
        this(x, y, Angle.fromRad(0));
    }

    public Pose2D(double xIn, double yIn) {
        this(Distance.fromIn(xIn), Distance.fromIn(yIn), Angle.fromRad(0));
    }

    public Pose2D withHeading(Angle heading) {
        return new Pose2D(x,y,heading);
    }
}