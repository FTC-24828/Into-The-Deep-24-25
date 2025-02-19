package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;

public class DepositExtensionSetState extends InstantCommand {
    public DepositExtensionSetState(Extension.State s) {
        super(() -> WRobot.getInstance().extension.setDepositState(s));
    }
}
