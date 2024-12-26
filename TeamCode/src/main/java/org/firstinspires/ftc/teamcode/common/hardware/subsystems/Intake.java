package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;

public class Intake implements WSubsystem {
    private WRobot robot = WRobot.getInstance();

    public void init(Servo intake_right, Servo intake_left, Servo claw_pivot, Servo claw) {
        intake_right.setDirection(Servo.Direction.FORWARD);
        intake_left.setDirection(Servo.Direction.REVERSE);

        robot.intake4B.setTargetPosition(0);
    }

    @Override
    public void update() {
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        robot.intake4B.write();
    }

    @Override
    public void reset() {

    }
}
