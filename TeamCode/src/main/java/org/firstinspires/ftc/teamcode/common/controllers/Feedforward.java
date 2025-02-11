package org.firstinspires.ftc.teamcode.common.controllers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Feedforward {
    public double Kf, current_output;

    public Feedforward(double Kf) {
        this.Kf = Kf;
    }

    public double calculate(double target) {
        current_output = target * Kf;
        return current_output;
    }

    public double getOutput() { return current_output; }

    public void set(double Kf) { this.Kf = Kf; }
}
