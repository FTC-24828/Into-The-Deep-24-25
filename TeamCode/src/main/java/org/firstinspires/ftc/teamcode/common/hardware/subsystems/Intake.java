package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.exceptions.ManualControlLockedException;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.WMath;

public class Intake implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();
    public enum State {TRANSFER, NEUTRAL, INTAKE, PUSH};
    private State state = State.TRANSFER;
    public State getState() { return state; }
    public void setState(State s) { state = s; }

    //four bar
    public enum BarState {RAISED, NEUTRAL, INTAKE, PUSH};
    public BarState bar_state;
    public double getBarPosition() {
        if (bar_state == BarState.NEUTRAL) return 0.5;
        else if (bar_state == BarState.RAISED) return 0.4;
        else if (bar_state == BarState.INTAKE) return 0.62;
        else return 0.75;
    }

    //wrist
    public enum WristState {INTAKE, TRANSFER}
    public WristState wrist_state;
    public double getWristPosition() { return wrist_state == WristState.INTAKE? 1 : 0; }

    //pivot
    public enum PivotState {HOME, MANUAL}
    public PivotState pivot_state;
    private double pivot_position = 0.5;
    public double getPivotPosition() { return pivot_state == PivotState.HOME ? 0.5: pivot_position; }
    public void setPivotPosition(double s) { pivot_position = s; }

    //claw
    public enum ClawState {OPEN, CLOSED}
    public boolean claw_lock = false;
    public ClawState claw_state;
    public void setClawState(@NonNull ClawState state) {
        claw_state = state;
    }
    public void toggleClawState() {
        if (claw_lock) return;
        claw_state = (claw_state == ClawState.CLOSED ? ClawState.OPEN : ClawState.CLOSED);
    }

    public void init(Servo bar0, Servo bar1, Servo wrist, Servo pivot, Servo claw) {
        bar0.setDirection(Servo.Direction.REVERSE);
        bar1.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);
        pivot.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.REVERSE);

        bar0.scaleRange(0, 1);
        bar1.scaleRange(0, 1);
        wrist.scaleRange(0, 1);
        pivot.scaleRange(0.45, 0.55);
        claw.scaleRange(0.1, 0.45);
    }

    @Override
    public void update() {
        switch (state) {
            case INTAKE:
                bar_state = BarState.INTAKE;
                wrist_state = WristState.INTAKE;
                pivot_state = PivotState.MANUAL;
                claw_lock = false;
                break;
            case NEUTRAL:
                bar_state = BarState.RAISED;
                wrist_state = WristState.INTAKE;
                pivot_state = PivotState.MANUAL;
                claw_lock = false;
                break;
            case TRANSFER:
                bar_state = BarState.NEUTRAL;
                wrist_state = WristState.TRANSFER;
                pivot_state = PivotState.HOME;
                pivot_position = 0.5;
                claw_lock = true;
                break;
            case PUSH:
                bar_state = BarState.PUSH;
                wrist_state = WristState.INTAKE;
                pivot_state = PivotState.MANUAL;
                claw_state = ClawState.OPEN;
                claw_lock = true;
                break;
            default:
                throw new IllegalArgumentException("Failed to find intake state");
        }

        pivot_position = WMath.clamp(pivot_position, 0, 1);
        robot.four_bar.setTargetPosition(getBarPosition());
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        robot.four_bar.write();
        robot.intake_wrist.setPosition(getWristPosition());
        robot.intake_pivot.setPosition(getPivotPosition());
        robot.intake_claw.setPosition(claw_state == ClawState.CLOSED ? 1 : 0);
    }


    @Override
    public void reset() {

    }
}
