package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.HashMap;

public class Arm implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum ArmState {UP, DOWN, SPECIMEN_IN, SPECIMEN_OUT, SAMPLE}
    public ArmState state = ArmState.DOWN;
    public HashMap<ArmState, Integer> arm_position = new HashMap<ArmState, Integer>() {{
        put(ArmState.UP, 0);
        put(ArmState.DOWN, 0);
        put(ArmState.SPECIMEN_IN, 0);
        put(ArmState.SPECIMEN_OUT, 0);
        put(ArmState.SAMPLE, 0);
    }};

    public double target_power = 0.0;

    public void init(DcMotorEx m0, DcMotorEx m1) {
        m0.setDirection(DcMotorSimple.Direction.FORWARD);
        m1.setDirection(DcMotorSimple.Direction.REVERSE);

        m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
        robot.arm_group.setPower(target_power);
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        robot.arm_group.write();
    }

    @Override
    public void reset() {

    }

    public void setTargetPower(double target) {
        target_power = WMath.clamp(target, -1, 1);
    }
}
