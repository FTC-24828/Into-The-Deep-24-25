package org.firstinspires.ftc.teamcode.common.hardware.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.controllers.PIDF;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.WMath;

public class SwervePod implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();
    private DcMotor motor;
    private CRServo servo;
    private WAnalogEncoder encoder;
    private double target_power = 0;
    private double target_heading = 0;
    private double prev_motor_power = 0;
    private double prev_servo_power = 0;
    private double current_heading;
    public boolean heading_override = false;
    public boolean resetting = false;

    public static double kP = 0.6;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.5;
    public PIDF heading_controller;

    public double HEADING_TO_SERVO_RATIO = 5/6d;
    public double HEADING_TOLERANCE = 0.02;
    public double POWER_TOLERANCE = 0.05;
    public double POWER_DEADZONE = 0.05;

    public void init(DcMotorEx m, CRServo s, WAnalogEncoder e) {
        motor = m;
        servo = s;
        servo.setPower(0);
        encoder = e;
        heading_controller = new PIDF(kP, kI, kD, kF, 0, 0);
        resetting = false;
    }

    public void read() {
        current_heading = WMath.wrapAngle(encoder.getPosition() * HEADING_TO_SERVO_RATIO);
    }

    public void periodic() {
    }

    public void write() {
        double error = wrappedError();
        if (Math.abs(target_power - prev_motor_power) > POWER_TOLERANCE) {
//            if (Math.abs(target_power - prev_power) > 0.12)
//                target_power = prev_power + 0.14 * Math.signum(target_power - prev_power);
//            if (Math.abs(WMath.wrapAngle(target_heading - current_heading)) > Math.PI/2)
//                target_power *= -1;
            target_power *= (Math.abs(WMath.wrapAngle(target_heading - current_heading)) > Math.PI/2 ?
                -1 : 1);

            motor.setPower(WMath.clamp(target_power, -1, 1));
            prev_motor_power = target_power;
        }

        double servo_power = heading_controller.calculate(error);
        if (Math.abs(servo_power) < POWER_DEADZONE) servo_power = 0;
        if (Math.abs(error) > HEADING_TOLERANCE
                && Math.abs(servo_power - prev_servo_power) > POWER_TOLERANCE
                && !heading_override) {
            servo.setPower(servo_power);
            prev_servo_power = servo_power;
        }
        else if (resetting) resetToZero();
    }

    public void reset() {
        resetToZero();
        target_power = 0;
    }

    public double wrappedError() {
        double error = WMath.wrapAngle(target_heading - current_heading);
        if (Math.abs(error) <= Math.PI / 2) return error;
        else if (error > Math.PI / 2) return error - Math.PI;
        return error + Math.PI;
    }

    public void setTargetHeading(double target) {
        target_heading = target;
    }

    public void setTargetPower(double target) {
        target_power = target;
    }

    public double getTargetHeading() {
        return target_heading;
    }

    public double getPodHeading() {
        return current_heading;
    }

    public double getServoPower() {
        return servo.getPower();
    }

    public void setServoPower(double power) {
        setHeadingOverride(power != 0);
        servo.setPower(power);
    }

    public void setWheelPower(double power) {
        motor.setPower(power);
    }

    public void setHeadingOverride(boolean bool) {
        heading_override = bool;
    }

    public void resetToZero() {
        resetting = true;
        if (Math.abs(encoder.getPosition()) < Math.PI / 4) {
            resetting = false;
            setServoPower(0);
        }
        else setServoPower(WMath.clamp(-encoder.getPosition(), -1, 1));
    }
}
