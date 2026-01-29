package org.firstinspires.ftc.teamcode.Util;

import java.util.ArrayList;

public class MathUtil {
    public static double piWraparound(double angle) {
        return Math.atan2(Math.sin(angle),Math.cos(angle));


    }

    public double getNormalized(ArrayList<Double> numbers, double target) {
        double normal = 1;
        for (double number : numbers) {
            if (number > target)
                normal = target / number;
        }
        return normal;
    }

    public ArrayList<Vector2D> getNormalizedVectors(ArrayList<Vector2D> vectors, double target) {
        ArrayList<Vector2D> normalizedVectors = new ArrayList<>();
        ArrayList<Double> magnitudes = new ArrayList<>();
        for (Vector2D vector : vectors) {
            magnitudes.add(vector.magnitude());
        }
        double normal = getNormalized(magnitudes, target);
        for (Vector2D vector : vectors) {
            normalizedVectors.add(vector.scale(normal));
        }
        return normalizedVectors;
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double angleDiff(double from, double to) {
        return piWraparound(to - from);
    }


}
