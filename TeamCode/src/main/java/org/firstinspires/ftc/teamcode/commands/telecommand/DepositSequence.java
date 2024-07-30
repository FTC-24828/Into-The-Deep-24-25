package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class DepositSequence extends ParallelCommandGroup {
    public DepositSequence() {
        super(
                new InstantCommand(() -> Global.setState(Global.State.SCORING)),
                new ArmSetStateCommand(Arm.ArmState.SCORING),
                new WristCommand(Intake.WristState.SCORING)
        );
    }
}
