package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;

public class Hang implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum HangState {HANGING, STOPPED, RETRACT}

    public double power = 0;
    public HangState hang_state = HangState.STOPPED;

    public void init(DcMotorEx hang_l, DcMotorEx hang_r, WServo hook_l, WServo hook_r) {
        hang_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang_l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang_l.setDirection(DcMotorSimple.Direction.FORWARD);

        hang_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang_r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang_r.setDirection(DcMotorSimple.Direction.FORWARD);

        hook_l.setDirection(Servo.Direction.REVERSE);
        hook_r.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void periodic() {
        robot.hang_actuator.setPower(power);
    }

    @Override
    public void read() {
        robot.arm_actuator.read();
    }

    @Override
    public void write() {
        robot.hang_actuator.write();
    }

    @Override
    public void reset() {
        robot.hang_actuator.setPower(0);
    }

    public void setHangPower(double power) {
        this.power = power;
    }

    public void setHangState(HangState state) {
        this.hang_state = state;
    }

    public void setHookPosition(double position) {
        robot.hook_left.setPosition(position);
        robot.hook_right.setPosition(position);
    }
}
