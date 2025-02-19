package org.firstinspires.ftc.teamcode.tests.hardware;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.Arrays;

@TeleOp (name = "Encoder Test", group = "Test")
public class EncoderTest extends OpMode {
    public WEncoder[][] encoder = new WEncoder[2][4];
    public int[][] offset = new int[2][4];
    public int hub = 1;
    public int index = 0;
    public boolean last_right = false;
    public boolean last_left = false;
    @Override
    public void init() {
        encoder [0][0] = new WEncoder(new MotorEx(hardwareMap, "motor00").encoder);
        encoder [0][1] = new WEncoder(new MotorEx(hardwareMap, "motor01").encoder);
        encoder [0][2] = new WEncoder(new MotorEx(hardwareMap, "motor02").encoder);
        encoder [0][3] = new WEncoder(new MotorEx(hardwareMap, "motor03").encoder);
        encoder [1][0] = new WEncoder(new MotorEx(hardwareMap, "motor10").encoder);
        encoder [1][1] = new WEncoder(new MotorEx(hardwareMap, "motor11").encoder);
        encoder [1][2] = new WEncoder(new MotorEx(hardwareMap, "motor12").encoder);
        encoder [1][3] = new WEncoder(new MotorEx(hardwareMap, "motor13").encoder);
        Arrays.fill(offset[0], 0);
        Arrays.fill(offset[1], 0);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper && !last_right) index++;
        else if (gamepad1.left_bumper && !last_left) index--;
        if (gamepad1.y) hub = 0;
        else if (gamepad1.a) hub = 1;
        index = (int) WMath.clamp(index, 0, 3);
        last_right = gamepad1.right_bumper;
        last_left = gamepad1.left_bumper;

        if (gamepad1.x) offset[hub][index] = encoder[hub][index].getPosition();

        telemetry.addData("motor", "hub: %d, index: %d", hub, index);
        telemetry.addData("offset", offset[hub][index]);
        telemetry.addData("position", encoder[hub][index].getPosition() - offset[hub][index]);
    }
}
