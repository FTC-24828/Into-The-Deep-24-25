package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.Queue;
import java.util.function.DoubleSupplier;

public class Intake implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum ClawState {OPEN, CLOSED}
    private ClawState claw_right_state;
    private ClawState claw_left_state;

    public enum ClawSide {LEFT, RIGHT, BOTH}

    public enum WristState {SCORING, FOLD, FLAT, LAUNCHING, MANUAL}
    public double target_position = 0.0;
    public DoubleSupplier wrist_angle;
    public double increment = 0;
    private double angle_offset = 0;      //NOTE: TUNE IF CLAW ANGLE IS WRONG
    public WristState wrist_state = WristState.FOLD;
    public double arm_target_angle;


    public void init(WServo wrist0, WServo wrist1, WServo claw0, WServo claw1) {
        wrist0.setDirection(Servo.Direction.FORWARD);
        wrist1.setDirection(Servo.Direction.REVERSE);
        angle_offset = robot.wrist0.getWritingOffset();

        claw0.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.FORWARD);

        claw0.scaleRange(0.4, 0.8);
        claw1.scaleRange(0.2, 0.6);

        wrist_angle = () -> (robot.wrist_actuator.getCurrentPosition() - angle_offset) * Math.PI ;
    }

    public void periodic() {
        //check pivot state and calculate target
        switch (wrist_state) {
            case FLAT:
                target_position = 0.2;
                break;

            case FOLD:
                target_position = 1;
                break;

            case SCORING:
                arm_target_angle = robot.arm.target_position / (3 * Global.TETRIX_MOTOR_TPR) * WMath.twoPI ;
                target_position = (WMath.twoPI / 3 - arm_target_angle) / 2 + 0.2;
                break;

            case LAUNCHING:
                target_position = -1;
                break;

            case MANUAL:
                break;

            default:
                target_position = robot.wrist_actuator.getCurrentPosition();
        }
        robot.wrist_actuator.setTargetPosition(target_position/2); //target_position should be from 1 to -1
    }

    public void read() {
        robot.wrist_actuator.read();
    }

    public void write() {
        robot.wrist_actuator.write();
    }

    public void reset() {
        setWristState(WristState.FOLD);
        setClawState(ClawSide.BOTH, ClawState.CLOSED);
    }

    public void setClawState(@NonNull ClawSide side, @NonNull ClawState state) {
        double position = (state == ClawState.OPEN) ? 0 : 1;    //NOTE: CHANGE IF CLAW IS INVERTED
        switch (side) {
            case BOTH:
                claw_left_state = state;
                robot.claw_left.setPosition(position);
                claw_right_state = state;
                robot.claw_right.setPosition(position);
                break;

            case LEFT:
                claw_left_state = state;
                robot.claw_left.setPosition(position);
                break;

            case RIGHT:
                claw_right_state = state;
                robot.claw_right.setPosition(position);
                break;
        }
    }

    public ClawState getClawState(@NonNull ClawSide side) {
        switch (side) {
            case LEFT:
                return claw_left_state;

            case RIGHT:
                return claw_right_state;

            //return OPEN if claws are in different states
            case BOTH:
                if (claw_left_state != claw_right_state) return ClawState.OPEN;
                return claw_left_state;

            default:
                throw new RuntimeException("getClawState method called with NULL");
        }
    }

    public void setWristState(@NonNull WristState state) {
        this.wrist_state = state;
    }

    public void setWristPosition(double position) {
        wrist_state = WristState.MANUAL;
        target_position = position;
    }
}
