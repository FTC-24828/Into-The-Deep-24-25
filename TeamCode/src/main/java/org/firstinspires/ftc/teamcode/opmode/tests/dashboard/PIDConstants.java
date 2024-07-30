package org.firstinspires.ftc.teamcode.opmode.tests.dashboard;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDConstants {
    public static double Kp = 0.0015;  // Proportional gain
    public static double Ki = 0.0001;   // Integral gain
    public static double Kd = 0.0002;   // Derivative gain
    public static double Kf = 0.2;
    public static double lim = 1000;
    public static double maxErr = 10;
}