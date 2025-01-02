package org.firstinspires.ftc.teamcode.common.hardware.drive.pathing;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.ArrayList;
import java.util.List;

public class PurePursuit {
    private final WRobot robot = WRobot.getInstance();
    private Localizer localizer = robot.localizer;

    //constraints
    public double r, max_vel, max_accel;

    public List<Path> path = new ArrayList<>();
    public List<Vector2D> intersection = new ArrayList<>();
    public int last_index = -1;

    public int current_goal;
    public int end_goal;
    public Pose goal = new Pose();
    private Pose current_position;

    public PurePursuit(double l, double m_vel, double m_accel) {
        r = l;
        max_vel = m_vel;
        max_accel = m_accel;
        current_position = localizer.getPose();
    }

    public Pose calculateGoal(int index) {
        Path p = path.get(index);
        current_position = localizer.getPose();

        //check if we have switched to a different path
        if (last_index != index) {
            last_index = index;
            current_goal = 0;
            end_goal = p.size() - 1;
        }

        //if the goal point is within the lookahead circle
        if (ptpDistance(current_position, p.get(current_goal)) < r && current_goal != end_goal)
            current_goal++;

        //check if current_goal is the start or end of the path
        if (current_goal == 0 || current_goal == end_goal)
            goal = p.get(current_goal);
        else {
            switch (findIntersections(p.get(current_goal).toVector2D(), p.get(current_goal-1).toVector2D())) {
                case 1:
                    goal = new Pose(intersection.get(0), p.get(current_goal).z);
                    break;
                case 2:
                    if (ptpDistance(p.get(current_goal).toVector2D(), intersection.get(0)) <
                            ptpDistance(p.get(current_goal).toVector2D(), intersection.get(1)))
                        goal = new Pose(intersection.get(0), p.get(current_goal).z);
                    else
                        goal = new Pose(intersection.get(1), p.get(current_goal).z);
                    break;
                default:
                    goal = p.get(current_goal);
            }
        }

        return goal;
    }

    private double ptpDistance(Vector2D a, Vector2D b) {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    private double ptpDistance(Pose a, Pose b) {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    private int findIntersections(Vector2D a, Vector2D b) {
        intersection.clear();
        int point_found = 0;

        //offset point vectors to the robot's origin
        double local_x1 = a.x - current_position.x;
        double local_x2 = b.x - current_position.x;
        double local_y1 = a.y - current_position.y;
        double local_y2 = b.y - current_position.y;

        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dr = ptpDistance(a, b);
        double det = local_x1 * local_y2 - local_x2 * local_y1;     //determinant
        double delta = r*r * dr*dr - det*det;                       //discriminant

        if (delta < 0) return 0;    //if there is no real solutions

        //bounds on solutions
        double minX = Math.min(a.x, b.x);
        double maxX = Math.max(a.x, b.x);
        double minY = Math.min(a.y, b.y);
        double maxY = Math.max(a.y, b.y);

        double deltaX = (Math.signum(dy) >= 0 ? 1 : -1) * dx * Math.sqrt(delta);
        double deltaY = Math.abs(dy) * Math.sqrt(delta);

        //solution 1
        double x1 = (dy * det + deltaX) / (dr*dr) + current_position.x;
        double y1 = (-dx * det + deltaY) / (dr*dr) + current_position.y;

        //check if solution is on the line segments between end points a and b
        if (minX <= x1 && x1 <= maxX && minY <= y1 && y1 <= maxY) {
            intersection.add(new Vector2D(x1, y1));
            point_found++;
        }
        if (delta == 0) return point_found;

        //solution 2 (if discriminant is not 0)
        double x2 = (dy * det - deltaX) / (dr*dr) + current_position.x;
        double y2 = (-dx * det - deltaY) / (dr*dr) + current_position.y;

        if (minX <= x2 && x2 <= maxX && minY <= y2 && y2 <= maxY) {
            intersection.add(new Vector2D(x2, y2));
            point_found++;
        }
        return point_found;
    }

    public void add(Path p) {
        path.add(p);
    }
}
