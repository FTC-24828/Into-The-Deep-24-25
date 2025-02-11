package org.firstinspires.ftc.teamcode.common.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {

    public final ElapsedTime timer = new ElapsedTime();

    public double max_accel, max_decel, max_vel, start, goal, position;

    public int state = 0;

    public MotionProfile(double m_accel, double m_vel, double m_decel) {
        this.max_accel = Math.abs(m_accel);
        this.max_decel = Math.abs(m_decel);
        this.max_vel = Math.abs(m_vel);
    }

    public MotionProfile(double m_accel, double m_vel) {
        this(m_accel, m_vel, m_accel);
    }

    public void set(double m_accel, double m_decel, double m_vel) {
        this.max_accel = Math.abs(m_accel);
        this.max_decel = Math.abs(m_decel);
        this.max_vel = Math.abs(m_vel);
    }

    public void update() {
        double distance = goal - start;
        double direction = Math.signum(distance) < 0? -1 : 1;

        double accel_t = max_vel / max_accel;
        double decel_t = max_vel / max_decel;

        double accel_d = 0.5 * max_accel * accel_t * accel_t * direction;
        double decel_d = max_vel * decel_t * direction
                - 0.5 * max_decel * decel_t * decel_t * direction;
        double vel = max_vel;

        //if the distance is not large enough to reach max velocity
        if (Math.abs(distance) < Math.abs(accel_d + decel_d)) {
            vel = Math.sqrt(Math.abs(2 * distance * max_accel * max_decel
                    * (max_accel + max_decel)))
                    / (max_accel + max_decel);
            accel_t = vel / max_accel;
            decel_t = vel / max_decel;
            accel_d = 0.5 * max_accel * accel_t * accel_t * direction;
            decel_d = vel * decel_t * direction
                    - 0.5 * max_decel * decel_t * decel_t * direction;
        }

        double cruise_d = distance - accel_d - decel_d;
        double cruise_t = Math.abs(cruise_d) / max_vel;
        double elapsed_time = timer.seconds();

        if (elapsed_time > accel_t + cruise_t + decel_t) {
            state = 3;
            position = goal;
        }
        else if (elapsed_time < accel_t) {
            state = 0;
            position = start + 0.5 * max_accel * elapsed_time * elapsed_time * direction;
        }
        else if (elapsed_time < accel_t + cruise_t) {
            state = 1;
            double cruise_time = elapsed_time - accel_t;
            position = start + accel_d + vel * cruise_time * direction;
        }
        else {
            state = 2;
            double deceleration_time = elapsed_time - accel_t - cruise_t;
            position = start + accel_d + cruise_d
                    + vel * deceleration_time * direction
                    - 0.5 * max_decel * deceleration_time * deceleration_time * direction;
        }
    }

    public void setEndPoints(double s, double g) {
        start = s;
        goal = g;
        state = 0;
        timer.reset();
    }
}
