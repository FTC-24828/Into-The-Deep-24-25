package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class IntakeExtensionSetState extends InstantCommand {
    public IntakeExtensionSetState(Extension.State s) {
        super(() -> WRobot.getInstance().extension.setIntakeState(s));
    }
}
