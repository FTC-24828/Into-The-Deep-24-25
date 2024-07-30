package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class ArmAdjustCommand extends InstantCommand {
    public ArmAdjustCommand(int increment) {
        super(
                () -> WRobot.getInstance().arm.incrementHeight(increment)
        );
    }
}
