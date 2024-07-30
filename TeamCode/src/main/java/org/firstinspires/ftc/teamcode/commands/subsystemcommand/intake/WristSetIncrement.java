package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class WristSetIncrement extends InstantCommand {
    public WristSetIncrement(double i) {
        super(
                () -> WRobot.getInstance().intake.increment = i
        );
    }
}
