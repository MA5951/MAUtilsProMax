package com.MAutils.Utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Circle2d {
    private final Translation2d center;
    private final double radius;

    public Circle2d(Translation2d center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    public Translation2d getCenter() {
        return center;
    }

    public double getRadius() {
        return radius;
    }

    /** בודק אם נקודה נמצאת בתוך העיגול */
    public boolean contains(Translation2d point) {
        return center.getDistance(point) <= radius;
    }

    /** מחזיר את המרחק של נקודה מהיקף העיגול */
    public double getDistance(Translation2d point) {
        return Math.max(0.0, center.getDistance(point) - radius);
    }

    /** מחזיר את הנקודה הקרובה ביותר על/בתוך העיגול */
    public Translation2d nearest(Translation2d point) {
        if (contains(point)) {
            return point;
        }
        // מציאת הנקודה על ההיקף בכיוון הנקודה המבוקשת
        Translation2d delta = point.minus(center);
        double distance = delta.getNorm();
        return center.plus(delta.times(radius / distance));
    }
}