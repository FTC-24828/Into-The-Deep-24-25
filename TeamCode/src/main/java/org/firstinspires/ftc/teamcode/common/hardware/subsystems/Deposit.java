package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;

import java.util.HashMap;

public class Deposit implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();
    public enum State {INTAKE, TRANSFER, SPECIMEN_SCORE, SAMPLE_SCORE, SAMPLE_DROP, NEUTRAL}
    private State state = State.TRANSFER;
    public State getState() { return state; }
    public void setState(State s) {
        state = s;
        crane_timer.reset();
    }

    //crane
    public enum CraneState {INTAKE, TRANSFER};
    public CraneState crane_state = CraneState.TRANSFER;
    public double getCranePosition() { return crane_state == CraneState.INTAKE? 0 : 1; }
    public ElapsedTime crane_timer = new ElapsedTime();

    //wrist
    public enum WristState {INTAKE, NEUTRAL, TRANSFER, SPECIMEN_SCORE, SAMPLE_SCORE, SAMPLE_DROP}
    public HashMap<WristState, Double> wrist_position = new HashMap<WristState, Double>() {{
        put(WristState.INTAKE, 0.92);
        put(WristState.NEUTRAL, 0.3);
        put(WristState.TRANSFER, 0.0);
        put(WristState.SPECIMEN_SCORE, 0.35);
        put(WristState.SAMPLE_SCORE, 0.7);
        put(WristState.SAMPLE_DROP, 0.85);
    }};
    public WristState wrist_state = WristState.NEUTRAL;
    public double getWristPosition() { return wrist_position.get(wrist_state); }
    public WristState setWristState(WristState s) {
        if (crane_timer.milliseconds() > 500 || wrist_state == s)
            return s;
        else return WristState.NEUTRAL;
    }

    //claw
    public enum ClawState {OPEN, CLOSED}
    public boolean claw_lock = false;
    public ClawState claw_state;
    public void setClawState(@NonNull ClawState state) {
        claw_state = state;
    }
    public void toggleClawState() {
        if (claw_lock) return;
        claw_state = claw_state == ClawState.CLOSED ? ClawState.OPEN : ClawState.CLOSED;
    }

    public void init(Servo crane, Servo wrist, Servo claw) {
        crane.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        crane.scaleRange(0, 1);
        wrist.scaleRange(0, 1);
        claw.scaleRange(0.5, 1);
    }

    @Override
    public void update() {
        switch (state) {
            case INTAKE:
                crane_state = CraneState.INTAKE;
                wrist_state = setWristState(WristState.INTAKE);
                claw_lock = false;
                break;
            case TRANSFER:
                crane_state = CraneState.TRANSFER;
                wrist_state = setWristState(WristState.TRANSFER);
                claw_lock = true;
                break;
            case NEUTRAL:
                crane_state = CraneState.TRANSFER;
                wrist_state = setWristState(WristState.NEUTRAL);
                claw_lock = true;
                break;
            case SPECIMEN_SCORE:
                crane_state = CraneState.TRANSFER;
                wrist_state = setWristState(WristState.SPECIMEN_SCORE);
                claw_lock = true;
                break;
            case SAMPLE_SCORE:
                crane_state = CraneState.INTAKE;
                wrist_state = setWristState(WristState.SAMPLE_SCORE);
                claw_lock = false;
                break;
            case SAMPLE_DROP:
                crane_state = CraneState.INTAKE;
                wrist_state = WristState.SAMPLE_DROP;
                break;
            default:
                throw new IllegalArgumentException("Failed to find deposit state");
        }
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        robot.deposit_crane.setPosition(getCranePosition());
        robot.deposit_wrist.setPosition(getWristPosition());
        robot.deposit_claw.setPosition(claw_state == ClawState.CLOSED ? 1 : 0);
    }

    @Override
    public void reset() {

    }

}
