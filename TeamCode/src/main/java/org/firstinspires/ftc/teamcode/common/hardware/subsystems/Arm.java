package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import androidx.collection.CircularArray;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.controllers.Feedforward;
import org.firstinspires.ftc.teamcode.common.controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.common.controllers.PIDF;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WActuator;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.DoubleRingBuffer;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.ArrayList;
import java.util.HashMap;

@Config
public class Arm implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    private final ElapsedTime dt_timer = new ElapsedTime();

    public double power = 0.0;
    public int target_position;
    public int arm_tick;
    public double arm_angle;
//    public DoubleRingBuffer tick_buffer = new DoubleRingBuffer(3);
    public double d_theta = 0;
    public double dd_theta = 0;
    public boolean reached = false;

    public static final double GEAR_RATIO = 25.0/20.0 * 60.0/15.0;
    public static final double ENCODER_RES = 537.7;
    public static int OFFSET_CONSTANT = 175;
    public static int ARM_OFFSET = 0;
    public static double POSITION_TOLERANCE = 10;

    public static double kP = 0.003;
    public static double kI = 0;
    public static double kD = 0.0003;
    public static double kF = 0.08;
    public static double max_accel = 10000;
    public static double max_decel = 1500;
    public static double max_vel = 20000;
    public static PIDF arm_pid = new PIDF(kP, kI, kD, 0.0);
    public static MotionProfile arm_profile = new MotionProfile(max_accel, max_vel, max_decel);
    public static Feedforward arm_ff = new Feedforward(kF);

    public enum ArmState {FRONT, BACK_AIM, BACK_PICKUP, SPECIMEN_IN, SPECIMEN_AIM, SAMPLE, RESET}
    public ArmState state = ArmState.FRONT;
    public HashMap<ArmState, Integer> arm_position = new HashMap<ArmState, Integer>() {{
        put(ArmState.FRONT, -175);
        put(ArmState.BACK_AIM, 1390);
        put(ArmState.BACK_PICKUP, 1510);
        put(ArmState.SPECIMEN_IN, 30);
        put(ArmState.SPECIMEN_AIM, 450);
        put(ArmState.SAMPLE, 740);
        put(ArmState.RESET, -10000);
    }};


    public void init(DcMotorEx m0, DcMotorEx m1) {
        m0.setDirection(DcMotorSimple.Direction.FORWARD);
        m1.setDirection(DcMotorSimple.Direction.REVERSE);

        m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_tick = robot.intSubscriber(Sensors.ARM_ENCODER) - ARM_OFFSET;
        target_position = arm_tick;
        arm_pid.tolerance = POSITION_TOLERANCE;
    }

    @Override
    public void update() {
//        d_theta = (tick_buffer.get(0)[0] - tick_buffer.get(1)[0])/tick_buffer.get(0)[1];
//        dd_theta = d_theta - (tick_buffer.get(1)[0] - tick_buffer.get(2)[0])/tick_buffer.get(1)[1];
//        arm_profile.update();
//        power = arm_pid.calculate(arm_tick, arm_profile.position);
        power = arm_pid.calculate(arm_tick, arm_position.get(state));
        if (state != ArmState.FRONT)
            power += arm_ff.calculate(Math.cos(arm_angle));
        reached = Math.abs(arm_tick - arm_position.get(state)) < POSITION_TOLERANCE;
        if (Math.abs(power) < 0.05) power = 0;
        if (state == ArmState.RESET) power = WMath.clamp(power, -0.4, 0.4);
        robot.arm_actuator.setPower(WMath.clamp(power, -1, 1));
    }

    @Override
    public void read() {
        arm_tick = robot.intSubscriber(Sensors.ARM_ENCODER) - ARM_OFFSET - OFFSET_CONSTANT;
        arm_angle = arm_tick / (GEAR_RATIO * ENCODER_RES) * WMath.twoPI;
        double[] v = {arm_tick, dt_timer.seconds()};
//        tick_buffer.add(v);
        dt_timer.reset();
    }

    @Override
    public void write() {
        robot.arm_actuator.write();
    }

    @Override
    public void reset() {

    }

    public void setState(ArmState s) {
        state = s;
        arm_profile.setEndPoints(arm_tick, arm_position.get(s));
    }

    public void resetArmOffset() {
        ARM_OFFSET = robot.intSubscriber(Sensors.ARM_ENCODER) + 80;
    }

    public void setArmOffset(int o) {
        ARM_OFFSET = o;
    }

    public void setTargetPower(double target) {
        power = WMath.clamp(target, -1, 1);
    }
}
