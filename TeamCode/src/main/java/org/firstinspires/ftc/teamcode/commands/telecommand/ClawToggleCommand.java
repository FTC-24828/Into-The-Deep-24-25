package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class ClawToggleCommand extends ConditionalCommand {
    public ClawToggleCommand (Intake.ClawSide side) {
        super (
            new ClawCommand(side, Intake.ClawState.OPEN),
            new ClawCommand(side, Intake.ClawState.CLOSED),
                () -> WRobot.getInstance().intake.getClawState(side) == Intake.ClawState.CLOSED
        );
    }
}
