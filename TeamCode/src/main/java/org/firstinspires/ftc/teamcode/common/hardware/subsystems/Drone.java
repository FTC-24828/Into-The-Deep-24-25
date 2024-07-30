package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;

public class Drone implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum DroneState {STORED, LAUNCHED};
    public DroneState drone_state = DroneState.STORED;

    public void init(WServo trigger) {
        trigger.setDirection(Servo.Direction.REVERSE);
        trigger.scaleRange(0, 1);

        robot.trigger.setPosition(0);
    }

    @Override
    public void periodic() {


    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
    }

    @Override
    public void reset() {
        resetDrone();
    }

    public void launchDrone() {
        drone_state = DroneState.LAUNCHED;
        robot.trigger.setPosition(1);
    }

    public void resetDrone() {
        drone_state = DroneState.STORED;
        robot.trigger.setPosition(0);
    }
}
