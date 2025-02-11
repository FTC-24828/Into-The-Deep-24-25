package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;

public class ArmResetCommand extends InstantCommand {
    public ArmResetCommand() {
        super(
                () -> WRobot.getInstance().arm.setState(Arm.ArmState.RESET)
        );
    }
}
