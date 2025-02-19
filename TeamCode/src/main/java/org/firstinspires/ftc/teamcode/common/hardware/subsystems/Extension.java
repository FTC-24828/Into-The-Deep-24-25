package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.controllers.PIDF;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.HashMap;

@Config
public class Extension implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum State {RETRACT, SPECIMEN, SPECIMEN_CLIP, EXTEND}
    public State intake_state = State.RETRACT;
    public State deposit_state = State.RETRACT;

    public HashMap<State, Integer> deposit_position = new HashMap<State, Integer>() {{
        put(State.RETRACT, 0);
        put(State.SPECIMEN, 500);
        put(State.SPECIMEN_CLIP, 960);
        put(State.EXTEND, 2090);
    }};

    public static double kP = 0.01;
    public static double kD = 0.0005;
    public static PIDF deposit_pid = new PIDF(kP, 0.0, kD, 0.0);

    public int intake_offset = 0;
    public int deposit_offset = 0;
    public int intake_tick;
    public int deposit_tick;

    public boolean intake_reached;
    public boolean deposit_reached;
    public ElapsedTime intake_timer;
    public ElapsedTime deposit_timer;

    public double intake_power = 0, deposit_power = 0;
    public static double INTAKE_FEEDFORWARD = 0.15;
    public static double DEPOSIT_FEEDFORWARD = 0.2;

    public void init(DcMotor[] motor) {
        motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[1].setDirection(DcMotorSimple.Direction.REVERSE);

        motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.read();
        intake_offset = robot.intSubscriber(Sensors.INTAKE_ENCODER);
        deposit_offset = robot.intSubscriber(Sensors.DEPOSIT_ENCODER);

        intake_timer = new ElapsedTime();
        deposit_timer = new ElapsedTime();
    }

    @Override
    public void update() {
        if (Math.abs(robot.motor[1].getVelocity()) < 10 && intake_timer.milliseconds() > 100)
            intake_reached = true;

        intake_power = intake_reached ? intake_feedforward()
                : intake_state == State.EXTEND ? 1 : -1;

        if (Math.abs(robot.motor[2].getVelocity()) < 10
                && deposit_timer.milliseconds() > 100) {
            deposit_reached = true;
            if (deposit_state == State.RETRACT)
                deposit_offset = robot.intSubscriber(Sensors.DEPOSIT_ENCODER);
        }
        deposit_power = deposit_feedforward() + (deposit_reached ? 0
                : deposit_pid.calculate(deposit_tick ,deposit_position.get(deposit_state)));

        if (Math.abs(intake_power) < 0.05) intake_power = 0;
        if (Math.abs(deposit_power) < 0.05) deposit_power = 0;
        robot.intake_extension.setPower(WMath.clamp(intake_power, -0.7, 1));
        robot.deposit_extension.setPower(WMath.clamp(deposit_power, -0.6, 1));
    }

    @Override
    public void read() {
        intake_tick = robot.intSubscriber(Sensors.INTAKE_ENCODER) - intake_offset;
        deposit_tick = robot.intSubscriber(Sensors.DEPOSIT_ENCODER) - deposit_offset;
    }

    @Override
    public void write() {
        robot.intake_extension.write();
        robot.deposit_extension.write();
    }

    @Override
    public void reset() {
        intake_offset = 0;
        deposit_offset = 0;
    }

    private double intake_feedforward() {
        if (intake_state == State.RETRACT) return -INTAKE_FEEDFORWARD;
        else if (intake_state == State.EXTEND) return INTAKE_FEEDFORWARD;
        else return 0;
    }

    private double deposit_feedforward() {
        if (deposit_state == State.RETRACT) return 0;
        else return DEPOSIT_FEEDFORWARD;
    }

    public void setIntakeState(State s) {
        if (s != intake_state) {
            intake_reached = false;
            intake_timer.reset();
            intake_state = s;
        }
    }

    public void setDepositState(State s) {
        if (s != deposit_state) {
            deposit_reached = false;
            deposit_timer.reset();
            deposit_state = s;
        }
    }
}