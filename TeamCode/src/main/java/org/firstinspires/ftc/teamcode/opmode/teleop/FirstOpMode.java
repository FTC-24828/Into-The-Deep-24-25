package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Jack")
public class FirstOpMode extends OpMode {

    public DcMotor FR;
    public DcMotor FL, BL, BR;

    public void init() {
        FR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        FL = hardwareMap.get(DcMotor.class, "motorRearRight");
        BL = hardwareMap.get(DcMotor.class, "lift");
        BR = hardwareMap.get(DcMotor.class, "podMiddle");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        double u = gamepad1.left_stick_y;
        FR.setPower(u);
        FL.setPower(u);
        BL.setPower(u);
        BR.setPower(u);

        telemetry.addData("left y", u);
        telemetry.update();
    }
}
