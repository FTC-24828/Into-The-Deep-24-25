package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class ClawCommand extends InstantCommand {
    public ClawCommand (Intake.ClawSide side, Intake.ClawState state) {
        super(
                () -> WRobot.getInstance().intake.setClawState(side, state)
        );
    }
}
