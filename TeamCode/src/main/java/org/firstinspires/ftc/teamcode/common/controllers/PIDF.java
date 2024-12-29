package org.firstinspires.ftc.teamcode.common.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tests.dashboard.PIDConstants;

public class PIDF {
    private final ElapsedTime timer = new ElapsedTime();

    public double Kp, Ki, Kd, Kf, intLim, tolerance;

    public double prev_estimate = 0, derivative = 0, last_error = 0, last_target, integral = 0;

    public double current_output;       //for debugging output

    /**
        @param Kp           proportional gain
        @param Ki           Integral gain
        @param Kd           derivative gain
        @param Kf           Low-pass filter gain
        @param lim          Integral limit
        @param tolerance    error tolerance
    */
    public PIDF (double Kp, double Ki, double Kd, double Kf, double lim, double tolerance) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.intLim = lim;
        this.tolerance = tolerance;
    }

    public PIDF (double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0, 0, 0);
    }
    public PIDF (double Kp, double Ki, double Kd, double Kf) {
        this(Kp, Ki, Kd, Kd, 1000, 0);
    }

    public void set(double Kp, double Ki, double Kd, double Kf, double lim, double maxErr) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.intLim = lim;
        this.tolerance = maxErr;
    }

    /**return and output based on error value*/
    public double calculate(double error) {
        if (Math.abs(error) < tolerance) return 0;  //in acceptable error range then exit

        //integral calculation with integral limit
        integral += (error * timer.seconds());
        if (Math.abs(integral) > intLim) integral = intLim * Math.signum(integral);

        //noise filter for derivative
        derivative = Kf * prev_estimate + (1 - Kf) * (error - last_error);
        prev_estimate = derivative;

        double output = Kp * error + Ki * integral + Kd * derivative / timer.seconds();

        //set error for next iteration
        last_error = error;
        current_output = output; //debugging purposes
        timer.reset();
        return output;
    }

    /**return an output based on the current state vs the target state*/
    public double calculate(double current, double target) {
        if (target != last_target) this.reset(target);
        double error = target - current;
        if (error <= tolerance && error >= -tolerance) return 0;  //acceptable error range exit

        //integral calculation with integral limit
        integral += (error * timer.seconds());
        if (Math.abs(integral) > intLim) integral = intLim * Math.signum(integral);

        //noise filter for derivative
        derivative = Kf * prev_estimate + (1 - Kf) * (error - last_error);
        prev_estimate = derivative;

        double output = Kp * error + Ki * integral + Kd * derivative / timer.seconds();

        //set error for next iteration
        last_error = error;
        current_output = output; //debugging purposes
        timer.reset();
        return output;
    }

    public void reset(double target) {
        integral = 0; last_error = 0; last_target = target;
        prev_estimate = 0; derivative = 0;
    }

    public void reset() {
        reset(0);
    }

    public String toString() {
        return "last error: " + last_error + "/n" +
                "last error: " + last_target + "/n" +
                "last error: " + integral + "/n" +
                "last error: " + derivative + "/n" +
                "last error: " + current_output + "/n";
    }

    public static PIDF create(double Kp, double Ki, double Kd, double Kf, double lim, double maxErr) {
        return new PIDF(Kp, Ki, Kd, Kf, lim, maxErr);
    }

    public static PIDF create(double Kp, double Ki, double Kd) {
        return new PIDF(Kp, Ki, Kd, 1, 1000, 0);
    }

    public static PIDF create(double Kp, double Ki, double Kd, double Kf, double lim) {
        return new PIDF(Kp, Ki, Kd, Kf, lim, 0);
    }

    public void set() {
        this.Kp = PIDConstants.Kp;
        this.Kd = PIDConstants.Kd;
        this.Ki = PIDConstants.Ki;
        this.Kf = PIDConstants.Kf;
        this.intLim = PIDConstants.lim;
        this.tolerance = PIDConstants.maxErr;
    }
}
