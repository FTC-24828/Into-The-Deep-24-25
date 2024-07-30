package org.firstinspires.ftc.teamcode.common.util;


public class WMath {
    public static final double twoPI = 2 * Math.PI;


    public static double clamp(double value, double min, double max) {
        if (max < min) throw new IllegalArgumentException("tried to call WMath.clamp with illegal arguments");
        return Math.min(max, Math.max(value, min));
    }

    /**Wraps angle and return an angle between pi to -pi inclusive*/
    public static double wrapAngle(double theta) {
        theta %= twoPI;
        theta = (theta + twoPI) % twoPI;
        if (theta > Math.PI) theta -= twoPI;
        return theta;
    }

    public static double getArcLength(double theta, double radius) {
        return theta * radius;
    }

    public static double max(double a, double b, double c) {
        return Math.max(Math.max(a, b), c);
    }
}
