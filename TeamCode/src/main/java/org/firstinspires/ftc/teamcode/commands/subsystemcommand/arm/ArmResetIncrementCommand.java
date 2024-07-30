package org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;

public class ArmResetIncrementCommand extends InstantCommand {
    public ArmResetIncrementCommand() {
        super (
                WRobot.getInstance().arm::resetIncrement
        );
    }
}
