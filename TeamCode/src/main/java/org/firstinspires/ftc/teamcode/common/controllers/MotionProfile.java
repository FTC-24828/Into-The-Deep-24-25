package org.firstinspires.ftc.teamcode.common.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime dt_timer = new ElapsedTime();
    private double start_time = 0;

    public double max_acceleration, max_deceleration, max_velocity;
    public double prev_position, velocity, acceleration;
    public double last_target = 0;
    public double starting_error = 0;

    public int state = 0;

    public MotionProfile(double m_accel, double m_decel, double m_vel) {
        this.max_acceleration = m_accel;
        this.max_deceleration = m_decel;
        this.max_velocity = m_vel;
    }

    public MotionProfile(double m_accel, double m_vel) {
        this(m_accel, m_accel, m_vel);
    }

    public void set(double m_accel, double m_decel, double m_vel) {
        this.max_acceleration = m_accel;
        this.max_deceleration = m_decel;
        this.max_velocity = m_vel;
    }

    public double update(double current, double target, double tolerance) {
        if (Math.abs(target - last_target) < tolerance)  this.reset(target);
        double error = target - current;
        if (starting_error == 0) starting_error = error;
        if (Math.abs(error) <= tolerance) {
            reset(0);
            dt_timer.reset();
            //return 0;
        }

        if (start_time == 0) start_time = timer.seconds();
        double elapsed_time = timer.seconds() - start_time;
        double dt = dt_timer.seconds();
//        double direction = Math.signum(error);
//        double theoretical_velocity_max = Math.abs(Math.sqrt(2.0 * starting_error / (1/max_acceleration + 1/max_deceleration)));
//        double total_time = 2.0 * Math.abs(starting_error) / theoretical_velocity_max;

        double acceleration_t = max_velocity / max_acceleration;
        double deceleration_t = max_velocity / max_deceleration;

        double acceleration_d = 0.5 * max_acceleration * acceleration_t;
        double deceleration_d = 0.5 * max_deceleration * deceleration_t;

        velocity = (current - prev_position) / dt;
        acceleration = 2.0 * (current - prev_position) / (dt * dt);

        //if the error is not large enough to reach max velocity
        if (error < acceleration_d + deceleration_d) {
            double theoretical_velocity_max = Math.sqrt(2.0 * Math.abs(starting_error) / (1/max_acceleration + 1/max_deceleration));
            acceleration_t = theoretical_velocity_max * 1 / max_acceleration;
            deceleration_t = theoretical_velocity_max * 1 / max_deceleration;
        }

        max_velocity = max_acceleration * acceleration_t;

        double cruise_d = error - 2 * acceleration_d;
        double cruise_t = cruise_d / max_velocity;

        double total_time = acceleration_t + cruise_t + deceleration_t;

        if (elapsed_time > total_time) {
            return error;
        }

        if (elapsed_time < acceleration_t) {
            return 0.5 * max_acceleration * acceleration_t * acceleration_t;
        }
        else if (elapsed_time < acceleration_t + cruise_t) {
            acceleration_d = 0.5 * max_acceleration * acceleration_t * acceleration_t;
            double cruise_time = elapsed_time - acceleration_t;
            return acceleration_d + max_velocity * cruise_time;
        }
        else {
            acceleration_d = 0.5 * max_acceleration * acceleration_t * acceleration_t;
            cruise_d = max_velocity * cruise_t;
            double deceleration_time = elapsed_time - acceleration_t - cruise_t;
            return acceleration_d + cruise_d + max_velocity * deceleration_time -
                    0.5 * max_acceleration * deceleration_time * deceleration_time;
        }
    }

    public void reset(double target) {
        last_target = target;
        start_time = 0;
        starting_error = 0;
    }
}
