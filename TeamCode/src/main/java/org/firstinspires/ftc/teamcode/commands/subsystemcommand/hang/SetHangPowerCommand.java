package org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class SetHangPowerCommand extends InstantCommand {
    public SetHangPowerCommand(double power) {
        super(
                () -> WRobot.getInstance().hang.setHangPower(power)
        );
    }
}
