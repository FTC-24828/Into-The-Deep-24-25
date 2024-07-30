package org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class ArmSetTargetCommand extends InstantCommand {
    public ArmSetTargetCommand(double target) {
        super(
                () -> WRobot.getInstance().arm.setTargetPosition(target)
        );
    }
}
