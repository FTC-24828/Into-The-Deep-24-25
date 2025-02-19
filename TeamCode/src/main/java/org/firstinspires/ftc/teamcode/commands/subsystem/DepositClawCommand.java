package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class DepositClawCommand extends InstantCommand {
    public DepositClawCommand(Deposit.ClawState s) {
        super (() -> WRobot.getInstance().deposit.setClawState(s));
    }
}
