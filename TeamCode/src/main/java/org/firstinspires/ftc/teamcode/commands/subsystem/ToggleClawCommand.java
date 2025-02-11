package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class ToggleClawCommand extends ConditionalCommand{
    public ToggleClawCommand() {
        super(
               new ClawCommand(Intake.ClawState.OPEN),
               new ClawCommand(Intake.ClawState.CLOSED),
               () -> WRobot.getInstance().intake.claw_state == Intake.ClawState.CLOSED
        );
    }
}
