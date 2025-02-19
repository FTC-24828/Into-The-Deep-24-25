package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class IntakeClawCommand extends InstantCommand {
    public IntakeClawCommand(Intake.ClawState s) {
        super (() -> WRobot.getInstance().intake.setClawState(s));
    }
}
