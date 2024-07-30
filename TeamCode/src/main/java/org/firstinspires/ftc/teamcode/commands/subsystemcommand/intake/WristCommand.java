package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class WristCommand extends InstantCommand {
    public WristCommand(Intake.WristState state) {
        super (
                () -> WRobot.getInstance().intake.setWristState(state)
        );
    }
}
