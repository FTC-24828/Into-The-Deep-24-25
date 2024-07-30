package org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class HookCommand extends InstantCommand {
    public HookCommand(double position) {
        super(
                () -> WRobot.getInstance().hang.setHookPosition(position)
        );
    }
}
