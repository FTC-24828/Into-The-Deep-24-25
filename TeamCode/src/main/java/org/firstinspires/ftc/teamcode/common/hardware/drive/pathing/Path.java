package org.firstinspires.ftc.teamcode.common.hardware.drive.pathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Path {
    public List<Pose> pose = new ArrayList<>();

    public Path() {

    }

    public Path(Pose... p) {
        Collections.addAll(pose, p);
    }

    public Path(List<Pose>... poses) {
        for (List<Pose> p : poses) {
            pose.addAll(p);
        }
    }

    public Path add(Pose... p) {
        Collections.addAll(pose, p);
        return this;
    }

    public Path add(List<Pose>... poses) {
        for (List<Pose> p : poses) {
            pose.addAll(p);
        }
        return this;
    }

    public Pose get(int index) {
        return pose.get(index);
    }

    public void connect(Path p) {
        this.pose.addAll(p.pose);
    }

    public int size() {
        return this.pose.size();
    }
}

