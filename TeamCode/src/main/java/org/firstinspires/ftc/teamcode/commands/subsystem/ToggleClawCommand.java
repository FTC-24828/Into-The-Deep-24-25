package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class ToggleClawCommand extends SequentialCommandGroup {
    public ToggleClawCommand() {
        super(new InstantCommand(() -> {
                   WRobot.getInstance().intake.toggleClawState();
                   WRobot.getInstance().deposit.toggleClawState();
               })
        );
    }
}
