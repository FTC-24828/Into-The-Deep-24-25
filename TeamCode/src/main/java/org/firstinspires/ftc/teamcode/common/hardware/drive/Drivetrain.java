package org.firstinspires.ftc.teamcode.common.hardware.drive;

import android.annotation.TargetApi;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.fasterxml.jackson.databind.Module;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

public class Drivetrain implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    private final double[] target_power = new double[4];
    public double[] target_heading = new double[4];
    public double[] current_heading = new double[4];
    public double TANGENT_TO_CENTER = Math.PI / 4;

    public ElapsedTime inactive_timer;

    public void init (DcMotorEx[] motor, CRServo[] servo, WAnalogEncoder[] encoder) {
//      set drivetrain properties

        motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[2].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[3].setDirection(DcMotorSimple.Direction.FORWARD);

        motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo[0].setDirection(DcMotorSimple.Direction.FORWARD);
        servo[1].setDirection(DcMotorSimple.Direction.FORWARD);
        servo[2].setDirection(DcMotorSimple.Direction.FORWARD);
        servo[3].setDirection(DcMotorSimple.Direction.FORWARD);

        encoder[0].setVoltageRange(3.3);
        encoder[1].setVoltageRange(3.3);
        encoder[2].setVoltageRange(3.3);
        encoder[3].setVoltageRange(3.3);

        encoder[0].setInverted(false);
        encoder[1].setInverted(false);
        encoder[2].setInverted(false);
        encoder[3].setInverted(false);

        encoder[0].setOffset(-1.401);
        encoder[1].setOffset(-0.923);
        encoder[2].setOffset(-1.527);
        encoder[3].setOffset(-0.814);

        normalHeading();

        for (int i=0; i<4; i++) {
            robot.pod[i].init(motor[i], servo[i], encoder[i]);
        }
    }

    public void periodic() {
        for (int i=0; i<4; i++) {
            robot.pod[i].setTargetPower(target_power[i]);
            robot.pod[i].setTargetHeading(target_heading[i]);
        }
    }

    public void read() {
        for (int i=0; i<4; i++) {
            robot.pod[i].read();
            current_heading[i] = robot.pod[i].getPodHeading();
        }
    }

    public void write() {
        for (SwervePod pod : robot.pod)
            pod.write();
    }

    public void reset() {
        normalHeading();
        for (SwervePod pod : robot.pod)
            pod.reset();
    }

    public void move(Vector2D v, double z) {
        move(v.x, v.y, z);
    }

    public void move(double x, double y, double z) {
        double angle = Math.atan2(x, y);
        double power = Math.hypot(x, y);
        if (inactive_timer == null) inactive_timer = new ElapsedTime();
        if (WMath.max(Math.abs(x), Math.abs(y), Math.abs(z)) < 0.1) {
//            if (inactive_timer.seconds() > 2.5) normalHeading();
            resetTargetPower();
            return;
        }

        inactive_timer.reset();

        double cos = Math.cos(TANGENT_TO_CENTER) * z;
        double sin = Math.sin(TANGENT_TO_CENTER) * z;

        Vector2D[] translated_vector = new Vector2D[4];
        translated_vector[0] = new Vector2D(x + cos, y - sin);      //  [0]_____[3]   +z is cw
        translated_vector[1] = new Vector2D(x + cos, y + sin);      //   |   ^   |
        translated_vector[2] = new Vector2D(x - cos, y + sin);      //   |   |   |       +x
        translated_vector[3] = new Vector2D(x - cos, y - sin);      //  [1]_____[2]   +y__|

        double max = 1;
        for (Vector2D v : translated_vector)
            max = Math.max(max, v.magnitude());

        if (max > 1) {
            translated_vector[0].scale(translated_vector[0].magnitude() / max);
            translated_vector[1].scale(translated_vector[1].magnitude() / max);
            translated_vector[2].scale(translated_vector[2].magnitude() / max);
            translated_vector[3].scale(translated_vector[3].magnitude() / max);
        }

//        if (Global.IS_AUTO) {
//            double correction = 12 / robot.getVoltage();
//            for (int i=0; i<target_power.length; i++) {
//                target_power = Math.abs(wheel_speed[i]) < 0.01 ?
//                        wheel_speed[i] * correction :
//                        (wheel_speed[i] + Math.signum(wheel_speed[i]) * 0.05) * correction;
//            }
//        }

        for (int i=0; i<4; i++) {
            target_power[i] = translated_vector[i].magnitude();
            target_heading[i] = translated_vector[i].direction();
        }
    }

    public void rotatePods(double power) {
        for (SwervePod pod: robot.pod) {
            pod.setServoPower(power);
        }
    }

    public void rotateWheels(double power) {
        for (SwervePod pod: robot.pod) {
            pod.setWheelPower(power);
        }
    }

    public void overrideHeading() {
        for (SwervePod pod: robot.pod) {
            pod.setHeadingOverride(true);
        }
    }

    public void setPodsHeading(double radian) {
        for (int i=0; i<4; i++) {
            target_heading[i] = radian;
        }
    }

    public void resetTargetPower() {
        for (int i=0; i<4; i++) {
            target_power[i] = 0;
        }
    }

    public void normalHeading() {
        target_heading[0] = -TANGENT_TO_CENTER + Math.PI / 2;
        target_heading[1] = TANGENT_TO_CENTER - Math.PI / 2;
        target_heading[2] = -TANGENT_TO_CENTER + Math.PI / 2;
        target_heading[3] = TANGENT_TO_CENTER - Math.PI / 2;
    }

    public void tangentHeading() {
        target_heading[0] = -TANGENT_TO_CENTER;
        target_heading[1] = TANGENT_TO_CENTER;
        target_heading[2] = -TANGENT_TO_CENTER;
        target_heading[3] = TANGENT_TO_CENTER;
    }
//
//    public String toString() {
//        return "W0: " + wheel_speed[0] + "W1: " + wheel_speed[1] + "W2: " + wheel_speed[2] + "W3: " + wheel_speed[3];
//    }
}
