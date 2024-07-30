package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.controllers.Feedforward;
import org.firstinspires.ftc.teamcode.common.controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.common.controllers.PIDF;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.function.DoubleSupplier;

@Config
public class Arm implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum ArmState { SCORING, FLAT, LAUNCHING, HANG }
    private ArmState arm_state = ArmState.FLAT;

    public DoubleSupplier arm_angle;
    public double target_position = 0;
    public double increment = 0;
    public double power = 0.0;

    public final double ARM_LENGTH = 17;

    //controllers
    public static double kP = 0.001;
    public static double kI = 0.0001;
    public static double kD = 0.0002;
    public static double kF = 0.65;

    public static PIDF arm_controller = new PIDF(kP, kI, kD, kF, 2000.0, 0);
    public static Feedforward arm_support = new Feedforward(0.2);
//    public static MotionProfile arm_profile = new MotionProfile(10, 2, 5);

    public Arm() {

    }

    public void init (DcMotorEx lift) {
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        arm_angle = () -> robot.arm_actuator.getCurrentPosition() / (3 * Global.TETRIX_MOTOR_TPR) * WMath.twoPI;
        target_position = robot.arm_actuator.getCurrentPosition();
    }

    public void periodic() {
        if (Global.IS_AUTO) {
            if (arm_state == ArmState.FLAT) target_position = robot.arm_actuator.getReadingOffset();
            power = arm_controller.calculate(robot.arm_actuator.getCurrentPosition(), target_position) +
                    arm_support.calculate(Math.cos(arm_angle.getAsDouble())) * ((arm_state == ArmState.FLAT) ? 0 : 1);
        } else {
            switch (arm_state) {
                case FLAT:
                    target_position = robot.arm_actuator.getReadingOffset() + increment;
                    break;

                case SCORING:
                    target_position = (double) Global.TETRIX_MOTOR_TPR + increment + 500;
                    break;

                case LAUNCHING:
                    target_position = (double) Global.TETRIX_MOTOR_TPR / 3;
                    break;
            }

            power = arm_controller.calculate(robot.arm_actuator.getCurrentPosition(), target_position) +
                    arm_support.calculate(Math.cos(arm_angle.getAsDouble())) * ((arm_state == ArmState.FLAT) ? 0 : 1);
        }

        robot.arm_actuator.setPower(power);
    }

    public void read() {
        robot.arm_actuator.read();
    }

    public void write() {
        robot.arm_actuator.write();
    }

    public void reset() {
        arm_controller.reset();
        setArmState(ArmState.FLAT);
    }

    public ArmState getArmState() {
        return arm_state;
    }

    public void setArmState(ArmState state) {
        arm_state = state;
    }

    public void setTargetPosition (double target_position){
        this.target_position = target_position;
    }

    public void incrementHeight(double increment) {
        this.increment -= increment;
        this.increment = WMath.clamp(this.increment, -200, 700);
    }

    public void resetIncrement() {
        this.increment = 0;
    }
}
