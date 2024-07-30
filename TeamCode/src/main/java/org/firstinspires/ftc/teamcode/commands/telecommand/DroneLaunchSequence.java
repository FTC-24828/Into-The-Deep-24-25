package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class DroneLaunchSequence extends SequentialCommandGroup {
    public DroneLaunchSequence() {
        super(
                new InstantCommand(() -> Global.setState(Global.State.LAUNCHING))
//                new ArmSetStateCommand(Arm.ArmState.LAUNCHING),
//                new WristCommand(Intake.WristState.LAUNCHING)
        );
    }
}
