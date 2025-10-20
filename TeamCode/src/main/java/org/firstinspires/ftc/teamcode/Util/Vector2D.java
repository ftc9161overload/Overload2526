package org.firstinspires.ftc.teamcode.Util; // Or your preferred package


import java.util.Locale;

/**
 * Represents a 2D vector with x and y components.
 * Provides common mathematical operations for 2D vectors.
 * This class is immutable; methods that would modify the vector return a new Vector2D instance.
 */
public class Vector2D {

    public final double x;
    public final double y;

    /**
     * A constant for the zero vector (0,0).
     */
    public static final Vector2D ZERO = new Vector2D(0, 0);

    /**
     * A constant for a unit vector along the X-axis (1,0).
     */
    public static final Vector2D UNIT_X = new Vector2D(1, 0);

    /**
     * A constant for a unit vector along the Y-axis (0,1).
     */
    public static final Vector2D UNIT_Y = new Vector2D(0, 1);


    /**
     * Default constructor, creates a zero vector (0,0).
     */
    public Vector2D() {
        this.x = 0.0;
        this.y = 0.0;
    }

    /**
     * Constructor to create a vector with specified x and y components.
     *
     * @param x The x-component of the vector.
     * @param y The y-component of the vector.
     */
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // --- Basic Vector Operations ---

    /**
     * Adds another vector to this vector.
     *
     * @param other The vector to add. Must not be null.
     * @return A new Vector2D representing the sum (this + other).
     */
    public Vector2D add(Vector2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot add a null vector.");
        }
        return new Vector2D(this.x + other.x, this.y + other.y);
    }

    /**
     * Subtracts another vector from this vector.
     *
     * @param other The vector to subtract. Must not be null.
     * @return A new Vector2D representing the difference (this - other).
     */
    public Vector2D subtract(Vector2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot subtract a null vector.");
        }
        return new Vector2D(this.x - other.x, this.y - other.y);
    }

    /**
     * Scales this vector by a scalar value (multiplication).
     *
     * @param scalar The scalar value to multiply by.
     * @return A new Vector2D representing the scaled vector (this * scalar).
     */
    public Vector2D scale(double scalar) {
        return new Vector2D(this.x * scalar, this.y * scalar);
    }

    /**
     * Negates this vector (flips its direction).
     * Equivalent to scaling by -1.
     *
     * @return A new Vector2D representing the negated vector (-this).
     */
    public Vector2D negate() {
        return new Vector2D(-this.x, -this.y);
    }

    // --- Magnitude and Normalization ---

    /**
     * Calculates the magnitude (length or norm) of this vector.
     *
     * @return The magnitude of the vector.
     */
    public double magnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    /**
     * Calculates the squared magnitude of this vector.
     * This is computationally cheaper than magnitude() if only comparisons are needed.
     *
     * @return The squared magnitude of the vector.
     */
    public double magnitudeSquared() {
        return this.x * this.x + this.y * this.y;
    }

    /**
     * Normalizes this vector, converting it to a unit vector (a vector with magnitude 1).
     * If the magnitude is zero, it returns a zero vector to avoid division by zero.
     *
     * @return A new Vector2D representing the normalized vector, or a zero vector if the original magnitude was zero.
     */
    public Vector2D normalize() {
        double mag = magnitude();
        if (mag == 0) { // Or use a small epsilon: Math.abs(mag) < 1e-9
            return Vector2D.ZERO;
        }
        return new Vector2D(this.x / mag, this.y / mag);
    }

    // --- Dot Product and Angle ---

    /**
     * Calculates the dot product of this vector with another vector.
     * The dot product can be used to find the angle between vectors or project one vector onto another.
     *
     * @param other The other vector. Must not be null.
     * @return The dot product (this · other).
     */
    public double dot(Vector2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot compute dot product with a null vector.");
        }
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Calculates the angle of this vector in radians with respect to the positive X-axis.
     * The angle is in the range (-PI, PI].
     *
     * @return The angle of the vector in radians.
     */
    public double angle() {
        return Math.atan2(this.y, this.x);
    }

    /**
     * Calculates the angle between this vector and another vector in radians.
     * The angle is in the range [0, PI].
     *
     * @param other The other vector. Must not be null.
     * @return The angle between the two vectors in radians. Returns NaN if either vector is a zero vector.
     */
    public double angleTo(Vector2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot compute angle to a null vector.");
        }
        double mag1Sq = this.magnitudeSquared();
        double mag2Sq = other.magnitudeSquared();

        if (mag1Sq == 0 || mag2Sq == 0) {
            return Double.NaN; // Angle is undefined if one vector is zero
        }
        double dotProduct = this.dot(other);
        double cosTheta = dotProduct / (Math.sqrt(mag1Sq * mag2Sq));

        // Clamp cosTheta to [-1, 1] to avoid floating point inaccuracies with acos
        cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));

        return Math.acos(cosTheta);
    }

    // --- Other Utility Methods ---

    /**
     * Calculates the distance between this vector (interpreted as a point) and another vector (point).
     *
     * @param other The other vector (point). Must not be null.
     * @return The distance between the two points.
     */
    public double distanceTo(Vector2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot compute distance to a null vector.");
        }
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculates the squared distance between this vector (point) and another vector (point).
     * Computationally cheaper than distanceTo() if only comparisons are needed.
     *
     * @param other The other vector (point). Must not be null.
     * @return The squared distance between the two points.
     */
    public double distanceSquaredTo(Vector2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot compute squared distance to a null vector.");
        }
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return dx * dx + dy * dy;
    }

// --- Other Utility Methods (Continued) ---

    /**
     * Rotates this vector by a given angle in radians around the origin (0,0).
     * Positive angle rotates counter-clockwise.
     *
     * @param angleRadians The angle of rotation in radians.
     * @return A new Vector2D representing the rotated vector.
     */
    public Vector2D rotate(double angleRadians) {
        double cosA = Math.cos(angleRadians);
        double sinA = Math.sin(angleRadians);
        double newX = this.x * cosA - this.y * sinA;
        double newY = this.x * sinA + this.y * cosA;
        return new Vector2D(newX, newY);
    }

    /**
     * Projects this vector onto another vector.
     *
     * @param other The vector to project onto. Must not be null.
     * @return A new Vector2D representing the projection of this vector onto the other.
     *         Returns a zero vector if the other vector has zero magnitude.
     */
    public Vector2D projectOnto(Vector2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot project onto a null vector.");
        }
        double otherMagSq = other.magnitudeSquared();
        if (Math.abs(otherMagSq) < 1e-9) { // Epsilon check for zero magnitude
            return Vector2D.ZERO;
        }
        double scaleFactor = this.dot(other) / otherMagSq;
        return other.scale(scaleFactor);
    }

    /**
     * Returns a new Vector2D that is perpendicular to this vector (rotated 90 degrees counter-clockwise).
     *
     * @return A new Vector2D perpendicular to this one.
     */
    public Vector2D perpendicular() {
        return new Vector2D(-this.y, this.x);
    }

    // --- Object Overrides ---

    /**
     * Checks if this vector is equal to another object.
     * Two vectors are considered equal if their x and y components are equal within a small tolerance.
     *
     * @param o The object to compare with.
     * @return True if the objects are equal, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Vector2D vector2D = (Vector2D) o;
        // Use a small epsilon for floating-point comparisons
        double epsilon = 1e-9; // Adjust tolerance as needed
        return Math.abs(vector2D.x - x) < epsilon &&
                Math.abs(vector2D.y - y) < epsilon;
    }

    /**
     * Generates a hash code for this vector.
     *
     * @return A hash code value for this object.
     */
    @Override
    public int hashCode() {
        // Use the standard Java utility for hashing multiple fields
        return java.util.Objects.hash(x, y);

        // Alternative manual implementation (often seen in older code or for specific needs):
        // long tempX = Double.doubleToLongBits(x); // Convert double to long bits for hashing
        // long tempY = Double.doubleToLongBits(y);
        // int result = (int) (tempX ^ (tempX >>> 32)); // Hash x
        // result = 31 * result + (int) (tempY ^ (tempY >>> 32)); // Combine with hash of y
        // return result;
    }

    /**
     * Returns a string representation of this vector.
     *
     * @return A string in the format "(x, y)".
     */
    @Override
    public String toString() {
        return String.format(Locale.US, "(%.3f, %.3f)", x, y); // Format to 3 decimal places
    }

    // --- Static Factory Methods ---

    /**
     * Creates a Vector2D from polar coordinates (magnitude and angle).
     *
     * @param magnitude The magnitude (length) of the vector.
     * @param angleRadians The angle of the vector in radians, measured counter-clockwise from the positive X-axis.
     * @return A new Vector2D instance.
     */
    public static Vector2D fromPolar(double magnitude, double angleRadians) {
        double x = magnitude * Math.cos(angleRadians);
        double y = magnitude * Math.sin(angleRadians);
        return new Vector2D(x, y);
    }

    /**
     * Linearly interpolates between two vectors.
     *
     * @param start The starting vector.
     * @param end The ending vector.
     * @param t The interpolation parameter (typically between 0.0 and 1.0).
     *          0.0 returns the start vector, 1.0 returns the end vector.
     * @return The interpolated Vector2D.
     */
    public static Vector2D lerp(Vector2D start, Vector2D end, double t) {
        if (start == null || end == null) {
            throw new IllegalArgumentException("Start and end vectors cannot be null for lerp.");
        }
        double x = start.x + (end.x - start.x) * t;
        double y = start.y + (end.y - start.y) * t;
        return new Vector2D(x, y);
    }
}