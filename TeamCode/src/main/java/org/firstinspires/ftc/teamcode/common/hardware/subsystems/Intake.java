package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;

public class Intake implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum ClawState {OPEN, CLOSED}
    public ClawState claw_state;

    public enum WristState {DOWN, MIDDLE, UP}
    public WristState wrist_state = WristState.MIDDLE;

    public double claw_position = 0.0;
    public double wrist_position = 0.5;

    public void init(Servo claw, Servo wrist) {
        claw.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);

        claw.scaleRange(0.35, 0.7);
        wrist.scaleRange(0.2, 0.8);
    }

    @Override
    public void update() {
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
           robot.wrist.setPosition(wrist_position);
    }

    public void setClawState(@NonNull ClawState state) {
        double position = (state == ClawState.CLOSED) ? 0 : 1;    //NOTE: CHANGE IF CLAW IS INVERTED
        robot.claw.setPosition(position);
        claw_state = state;
    }

    public void setWristState(@NonNull WristState state) {
        switch (state) {
            case UP:
                wrist_position = 1;
                break;
            case DOWN:
                wrist_position = 0;
                break;
            default:
                wrist_position = 0.5;
        }
        wrist_state = state;
    }

    public ClawState getClawState() {
        return claw_state;
    }


    @Override
    public void reset() {

    }
}
