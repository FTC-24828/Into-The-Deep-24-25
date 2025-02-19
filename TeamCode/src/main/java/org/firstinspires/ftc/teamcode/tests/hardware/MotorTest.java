package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.util.WMath;

@TeleOp (name = "Motor Test", group = "Test")
public class MotorTest extends OpMode {
    public DcMotor[][] motor = new DcMotor[2][4];
    public int hub = 1;
    public int index = 0;
    public boolean last_right = false;
    public boolean last_left = false;
    @Override
    public void init() {
        motor [0][0] = hardwareMap.get(DcMotor.class, "motor00");
        motor [0][1] = hardwareMap.get(DcMotor.class, "motor01");
        motor [0][2] = hardwareMap.get(DcMotor.class, "motor02");
        motor [0][3] = hardwareMap.get(DcMotor.class, "motor03");
        motor [1][0] = hardwareMap.get(DcMotor.class, "motor10");
        motor [1][1] = hardwareMap.get(DcMotor.class, "motor11");
        motor [1][2] = hardwareMap.get(DcMotor.class, "motor12");
        motor [1][3] = hardwareMap.get(DcMotor.class, "motor13");
    }

    @Override
    public void loop() {
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1.right_bumper && !last_right) index++;
        else if (gamepad1.left_bumper && !last_left) index--;
        if (gamepad1.y) hub = 0;
        else if (gamepad1.a) hub = 1;
        index = (int) WMath.clamp(index, 0, 3);
        motor[hub][index].setPower(power);
        last_right = gamepad1.right_bumper;
        last_left = gamepad1.left_bumper;

        telemetry.addData("motor", "hub: %d, index: %d", hub, index);
        telemetry.addData("power", power);
    }
}
