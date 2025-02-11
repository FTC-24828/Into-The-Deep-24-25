package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class ArmSetPowerCommand extends InstantCommand {
    public ArmSetPowerCommand (double target) {
        super(() -> WRobot.getInstance().arm.setTargetPower(target));
    }
}
