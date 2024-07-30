package org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;

public class ArmSetStateCommand extends InstantCommand {
    public ArmSetStateCommand(Arm.ArmState state) {
        super (
                () -> WRobot.getInstance().arm.setArmState(state)
        );
    }
}
