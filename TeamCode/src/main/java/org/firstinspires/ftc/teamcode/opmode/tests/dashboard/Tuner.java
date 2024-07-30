package org.firstinspires.ftc.teamcode.opmode.tests.dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.teamcode.common.controllers.PIDF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp (name = "PIDTuner", group = "Utility")
public class Tuner extends OpMode {

    // Declare OpMode members
    ElapsedTime runTime = new ElapsedTime();

    private BNO055IMU imu;
    DcMotor motor;
    Camera camera;
    boolean USE_WEBCAM;
    AprilTagProcessor aprilTag;
    VisionPortal vision;

    static final int TPR = 1440;

    int targetPosition = 0;

    public void init() {
        Gyroscope imu = hardwareMap.get(Gyroscope.class, "imu");

        motor = hardwareMap.get(DcMotor.class, "lift");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Camera camera;
        boolean USE_WEBCAM;
        AprilTagProcessor aprilTag;
        VisionPortal vision;


        telemetry.addData("Status", "Initialized");
    }

    PIDF armControl = PIDF.create(0,0,0);

    @Override
    public void start() {
    }

    public void loop() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armControl.set(new PIDConstants());

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setPower(0);

        if (gamepad1.y) {
            targetPosition = 6 * TPR / 5;
        }

        if (gamepad1.a) {
            targetPosition = 0;
        }


        motor.setPower(armControl.calculate(motor.getCurrentPosition(), targetPosition));

        telemetry.addData("motor tick", motor.getCurrentPosition());
        telemetry.addData("Baseline", 0);
        telemetry.addData("PID error", targetPosition - motor.getCurrentPosition());
        telemetry.addData("PID filter",  armControl.derivative);
        telemetry.addData("PID Output x 1000", armControl.current_output * 1000);
        telemetry.addData("PID integral", armControl.integral);

        telemetry.update();
    }
}