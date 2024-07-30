package org.firstinspires.ftc.teamcode.opmode.tests.system;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "hang check", group = "Utility")
public class HangCheck extends OpMode {
    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "hang");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            motor.setPower(1);
        }
        if (gamepad1.a) {
            motor.setPower(0);
        }
        if (gamepad1.b) {
            motor.setPower(-1);
        }
    }
}
