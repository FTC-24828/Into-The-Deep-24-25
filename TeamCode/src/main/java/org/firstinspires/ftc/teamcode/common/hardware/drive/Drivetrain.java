package org.firstinspires.ftc.teamcode.common.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

public class Drivetrain implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();
    public double[] wheel_speed = new double[4];
    private final double[] prev_speed = new double[4];

    public void init (DcMotorEx[] motor) {
//      set drivetrain properties
        motor[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[3].setDirection(DcMotorSimple.Direction.REVERSE);

        motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.middle_port.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.middle_port.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (int i=0; i<4; i++) {
            prev_speed[i] = 0;
        }
    }

    public void periodic() {

    }

    public void read() {
    }

    public void write() {
        for (int i=0; i<4; i++) {
            if (!Global.IS_AUTO) {
                if (Math.abs(wheel_speed[i] - prev_speed[i]) > 0.05) {
                    if (Math.abs(wheel_speed[i] - prev_speed[i]) > 0.12)
                        wheel_speed[i] = prev_speed[i] + 0.14 * Math.signum(wheel_speed[i] - prev_speed[i]);
                    robot.motor[i].setPower(WMath.clamp(wheel_speed[i], -1, 1));
                    prev_speed[i] = wheel_speed[i];
                }
            }
            else {
                robot.motor[i].setPower(WMath.clamp(wheel_speed[i], -1, 1));
                prev_speed[i] = wheel_speed[i];
            }
        }
    }

    public void reset() {
    }

    public void move(Pose pose) {
        move(pose.x, pose.y, pose.z) ;
    }

    public void move(Vector2D v, double z) {
        move(v.x, v.y, z);
    }

    public void move(double x, double y, double z) {
        double angle = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double cos = Math.cos(angle - Math.PI/4);
        double sin = Math.sin(angle - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double[] speed = {
                power * sin / max,
                power * cos / max,
        };

        wheel_speed[0] = speed[0] - z;
        wheel_speed[1] = speed[1] - z;
        wheel_speed[2] = speed[0] + z;
        wheel_speed[3] = speed[1] + z;

        if (Global.IS_AUTO) {
            double correction = 12 / robot.getVoltage();
            for (int i=0; i<wheel_speed.length; i++) {
                wheel_speed[i] = Math.abs(wheel_speed[i]) < 0.01 ?
                        wheel_speed[i] * correction :
                        (wheel_speed[i] + Math.signum(wheel_speed[i]) * 0.05) * correction;
            }
        }

        max = 1;
        for (double wheelSpeed : wheel_speed) max = Math.max(max, Math.abs(wheelSpeed));


        if (max > 1) {
            wheel_speed[0] /= max;
            wheel_speed[1] /= max;
            wheel_speed[2] /= max;
            wheel_speed[3] /= max;
        }
    }

    public String toString() {
        return "W0: " + wheel_speed[0] + "W1: " + wheel_speed[1] + "W2: " + wheel_speed[2] + "W3: " + wheel_speed[3];
    }
}
