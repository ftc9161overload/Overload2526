package org.firstinspires.ftc.teamcode.Util;

import java.util.ArrayList;

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
}
