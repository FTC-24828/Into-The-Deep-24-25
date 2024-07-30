package org.firstinspires.ftc.teamcode.common.hardware.drive.pathing;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

public class Pose {
    public double x, y, z;

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(Vector2D v, double z) {
        this(v.x, v.y, z);
    }

    public Pose(double x, double y, double z) {
        this.x = x; this.y = y; this.z = WMath.wrapAngle(z);
    }

    public Pose add(Pose p) {
        return new Pose(this.x + p.x, this.y + p.y, this.z + z);
    }

    public Pose subtract(Pose p) {
        return new Pose(this.x - p.x, this.y - p.y, this.z - p.z);
    }

    public Pose multiply(Pose p) {
        return new Pose(this.x * p.x, this.y * p.y, this.z * p.z);
    }

    public Pose divide(Pose p) {
        return new Pose(this.x / p.x, this.y / p.y, this.z / p.z);
    }

    public Vector2D toVector2D() {
        return new Vector2D(x, y);
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        return String.format("x: %.2f, y: %.2f, z: %.2f", x, y, z);
    }
}
