package org.firstinspires.ftc.teamcode.common.hardware.drive.pathing;

import org.firstinspires.ftc.teamcode.common.util.Vector2D;

import java.util.ArrayList;
import java.util.List;


public class Spline {
    public Pose start, end;
    public Vector2D  t_start, t_end;

    public enum EasingType {
        LINEAR, TANGENT, INOUT
    }

    public Spline(Pose s, Pose e, Vector2D t_s, Vector2D t_e) {
        start = s;
        end = e;
        t_start = t_s;
        t_end = t_e;
    }

    public List<Pose> generate(int interval, EasingType type) {
        List<Pose> pose = new ArrayList<>();
        int i = 0;
        while (i <= interval) {
            double t = (double) i / interval;
            double h00 = 2*t*t*t - 3*t*t + 1;
            double h10 = t*t*t - 2*t*t + t;
            double h01 = -2*t*t*t + 3*t*t;
            double h11 = t*t*t - t*t;

            pose.add(new Pose(new Vector2D(
                    h00 * start.x + h10 * t_start.x + h01 * end.x + h11 * t_end.x,
                    h00 * start.y + h10 * t_start.y + h01 * end.y + h11 * t_end.y),
                    i == interval? end.z : start.z
            ));
            i++;
        }
        return pose;
    }
}
