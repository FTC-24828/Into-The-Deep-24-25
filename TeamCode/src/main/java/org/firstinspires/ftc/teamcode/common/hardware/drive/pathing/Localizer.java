package org.firstinspires.ftc.teamcode.common.hardware.drive.pathing;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.function.DoubleSupplier;

@Config
public class Localizer {
    private final WRobot robot = WRobot.getInstance();

    private Pose start;
    private Pose pose;

    public static double WHEEL_RADIUS = 0.952;
    public static double TRACK_WIDTH = 7.1;
    public static double MIDDLE_OFFSET = 4.563;
    public static double SIDES_OFFSET = 2;

    private DoubleSupplier left, middle, right;
    private double _left, _middle, _right, _theta = 0.0;

    public double d_left, d_middle, d_right, d_theta;

    double local_dx, local_dy;

    public Localizer(Pose pose) {
        start = pose;
        this.pose = start;
    }

    public void init() {
        left = () -> robot.doubleSubscriber(Sensors.Encoder.POD_LEFT);
        middle = () -> robot.doubleSubscriber(Sensors.Encoder.POD_MIDDLE);
        right = () -> robot.doubleSubscriber(Sensors.Encoder.POD_RIGHT);
        read();
    }

    public void read() {
        _left = left.getAsDouble();
        _middle = middle.getAsDouble();
        _right = right.getAsDouble();
        _theta = pose.z;
    }

    public void update() {
        d_left = ticksToInches(left.getAsDouble() - _left);
        d_middle = ticksToInches(middle.getAsDouble() - _middle);
        d_right = ticksToInches(right.getAsDouble() - _right);

        d_theta = (d_left - d_right) / TRACK_WIDTH;
        pose.z = WMath.wrapAngle(pose.z + d_theta);

        local_dx = d_middle - MIDDLE_OFFSET * d_theta;
        local_dy = (d_left + d_right) * 0.5;
        Vector2D translated = new Vector2D(local_dx, local_dy, -pose.z);
        pose.x += translated.x;
        pose.y += translated.y;
        read();
    }

    private double ticksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / Global.GOBILDA_ENCODER_TPR;
    }

    public void reset(Pose p) {
        setPose(p);
        setStart(p);
    }

    public void reset() {
        reset(new Pose());
        robot.drivetrain.reset();
    }

    public void setStart(Pose pose) {
        start = pose;
    }

    public void setPose(Pose p) {
        this.pose = p;
    }

    public Pose getPose() {
        return pose;
    }
}