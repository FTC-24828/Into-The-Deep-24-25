package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class IntakeSetState extends InstantCommand {
    public IntakeSetState(Intake.State s) {
        super(() -> WRobot.getInstance().intake.setState(s));
    }
}
