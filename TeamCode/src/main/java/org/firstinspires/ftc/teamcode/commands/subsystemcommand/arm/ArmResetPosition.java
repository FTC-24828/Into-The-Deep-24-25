package org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class ArmResetPosition extends InstantCommand {
    public ArmResetPosition() {
        super (
                () -> new ArmSetTargetCommand((double) WRobot.getInstance().arm_actuator.getReadingOffset())
        );
    }
}
