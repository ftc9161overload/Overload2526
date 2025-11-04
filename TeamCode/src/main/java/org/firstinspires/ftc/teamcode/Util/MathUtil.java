package org.firstinspires.ftc.teamcode.Util;

public class MathUtil {
    public static double piWraparound(double angle) {
        angle %= (2 * Math.PI);
        if (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        } else if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;


    }
}
