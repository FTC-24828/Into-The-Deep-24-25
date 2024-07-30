package org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Hang;

public class SetHangState extends InstantCommand {
    public SetHangState(Hang.HangState state) {
        super(
                () -> WRobot.getInstance().hang.setHangState(state)
        );
    }
}
