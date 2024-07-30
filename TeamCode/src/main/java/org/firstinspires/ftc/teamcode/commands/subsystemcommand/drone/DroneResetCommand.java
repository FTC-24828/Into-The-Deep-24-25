package org.firstinspires.ftc.teamcode.commands.subsystemcommand.drone;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class DroneResetCommand extends InstantCommand {
    public DroneResetCommand() {
        super (
                WRobot.getInstance().drone::resetDrone
        );
    }
}
