package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.Arrays;

@TeleOp (name = "Servo Test", group = "Test")
public class ServoTest extends OpMode {
    public Servo[][] servo= new Servo[2][6];
    public int hub = 1;
    public int index = 0;
    double[][] position = new double[2][6];
    public boolean last_right = false;
    public boolean last_left = false;
    @Override
    public void init() {
//        servo [0][0] = hardwareMap.get(Servo.class, "servo00");
//        servo [0][1] = hardwareMap.get(Servo.class, "servo01");
//        servo [0][2] = hardwareMap.get(Servo.class, "servo02");
//        servo [0][3] = hardwareMap.get(Servo.class, "servo03");
        servo [0][4] = hardwareMap.get(Servo.class, "servo04");
        servo [0][5] = hardwareMap.get(Servo.class, "servo05");

        servo [1][0] = hardwareMap.get(Servo.class, "servo10");
        servo [1][1] = hardwareMap.get(Servo.class, "servo11");
        servo [1][2] = hardwareMap.get(Servo.class, "servo12");
        servo [1][3] = hardwareMap.get(Servo.class, "servo13");
        servo [1][4] = hardwareMap.get(Servo.class, "servo14");
        servo [1][5] = hardwareMap.get(Servo.class, "servo15");

        Arrays.fill(position[0], 0.5);
        Arrays.fill(position[1], 0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper && !last_right) index++;
        else if (gamepad1.left_bumper && !last_left) index--;
        if (gamepad1.y) hub = 0;
        else if (gamepad1.a) hub = 1;
        index = (int) WMath.clamp(index, 0, 5);

        if (gamepad1.dpad_up) position[hub][index] = 0.5 + 0.5 * gamepad1.right_stick_y;
        servo[hub][index].setPosition(position[hub][index]);
        last_right = gamepad1.right_bumper;
        last_left = gamepad1.left_bumper;

        telemetry.addData("servo", "hub: %d, index: %d", hub, index);
        telemetry.addData("dpad down pressed", gamepad1.dpad_up);
        telemetry.addData("position", position[hub][index]);
    }
}
