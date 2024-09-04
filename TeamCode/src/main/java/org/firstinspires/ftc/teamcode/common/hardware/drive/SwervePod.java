package org.firstinspires.ftc.teamcode.common.hardware.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

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
    private double m_target = 0;
    private double m_current = 0;
    private double s_current = 0;
    private double target_heading = 0;
    private double current_heading;
    public boolean heading_override = false;
    public boolean resetting = false;

    public PIDF heading_controller;
    public static double kP = 0.6;
    public static double kI = 0;
    public static double kD = 0.01;
    public static double kF = 0;

    public double HEADING_TO_SERVO_RATIO = 1.0;
    public double HEADING_TOLERANCE = 0.02;
    public double POWER_TOLERANCE = 0.1;
    public double POWER_DEADZONE = 0.05;
    public double MAX_MOTOR = 1;
    public double MAX_SERVO = 1;

    public void init(DcMotorEx m, CRServo s, WAnalogEncoder e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500.0, 2500.0, 5000.0));
        servo.setPower(0);

        encoder = e;
        heading_controller = new PIDF(kP, kI, kD, kF);
        resetting = false;
    }

    public void read() {
        current_heading = WMath.wrapAngle(encoder.getPosition() * HEADING_TO_SERVO_RATIO);
    }

    public void update() {}

    public void write() {
        double error = minError();
        if (Math.abs(error) > Math.PI/4.0 || Math.abs(m_target) < POWER_DEADZONE) m_target = 0;
        else m_target *= (Math.abs(WMath.wrapAngle(target_heading - current_heading)) > Math.PI/2 ? -1 : 1);
        if (Math.abs(m_target - m_current) > POWER_TOLERANCE
            || (m_target == 0 && m_current != 0)) {
            if (m_target != 0)
                m_target = m_current + 0.05 * Math.signum(m_target - m_current);
            motor.setPower(WMath.clamp(m_target, -MAX_MOTOR, MAX_MOTOR));
            m_current = m_target;
        }

        double s_target = heading_controller.calculate(0.0, error);
        if (Math.abs(s_target) < POWER_DEADZONE || Math.abs(error) <= HEADING_TOLERANCE)
            s_target = 0;
        if ((Math.abs(s_target - s_current) > POWER_TOLERANCE
                || (s_target == 0 && s_current != 0))
                && !heading_override) {
            servo.setPower(WMath.clamp(s_target, -MAX_SERVO, MAX_SERVO));
            s_current = s_target;
        }
        else if (resetting) resetToZero();
    }

    public void reset() {
        resetToZero();
        m_target = 0;
    }

    public double minError() {
        double error = WMath.wrapAngle(target_heading - current_heading);
        if (Math.abs(error) <= Math.PI / 2) return error;
        else if (error > Math.PI / 2) return error - Math.PI;
        return error + Math.PI;
    }

    public void setTargetHeading(double target) {
        target_heading = target;
    }

    public void setMotorTargetPower(double target) {
        m_target = target;
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
