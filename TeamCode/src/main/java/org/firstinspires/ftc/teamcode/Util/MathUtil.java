package org.firstinspires.ftc.teamcode.Util;

public class MathUtil {
    public static double piWraparound(double angle) {
        angle %= Math.PI*2;
        if (angle < 0) {
            angle += Math.PI;
        }

        angle -= Math.PI;

        return angle;


    }
}
