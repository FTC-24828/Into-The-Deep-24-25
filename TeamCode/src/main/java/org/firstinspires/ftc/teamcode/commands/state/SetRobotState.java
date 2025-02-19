package org.firstinspires.ftc.teamcode.commands.state;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Global;

public class SetRobotState extends InstantCommand {
    public SetRobotState(Global.State s) {
        super (() -> Global.STATE = s);
    }
}
