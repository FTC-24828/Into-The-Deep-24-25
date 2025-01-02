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

    public static double WHEEL_RADIUS = 0.942882;
    public static double X_OFFSET = 1.45033;
    public static double Y_OFFSET = -4.48733;
    public static double THETA_OFFSET = 0.0;

    private DoubleSupplier x, y;
    private double _x, _y, _theta = 0.0;

    public double dx, dy, dtheta;

    double local_dx, local_dy;

    public Localizer(Pose pose) {
        start = pose;
        this.pose = start;
    }

    public void init() {
        y = () -> robot.doubleSubscriber(Sensors.POD_Y);
        x = () -> robot.doubleSubscriber(Sensors.POD_X);
        read();
    }

    public void read() {
        _x= x.getAsDouble();
        _y= y.getAsDouble();
        _theta = robot.getYaw() + THETA_OFFSET;
    }

    public void update() {
        dx = ticksToInches(x.getAsDouble() - _x);
        dy = ticksToInches(y.getAsDouble() - _y);
        pose.z = WMath.wrapAngle(robot.getYaw() + THETA_OFFSET);
        dtheta = WMath.wrapAngle(pose.z - _theta);

        local_dx = dx - dtheta * X_OFFSET;
        local_dy = dy - dtheta * Y_OFFSET;

        Vector2D translated = new Vector2D(local_dx, local_dy, pose.z);
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

    public void setThetaOffset(double o) {
        THETA_OFFSET = o;
    }
}