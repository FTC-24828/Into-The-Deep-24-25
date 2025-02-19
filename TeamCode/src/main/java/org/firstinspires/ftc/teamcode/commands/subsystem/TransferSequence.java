package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.state.NeutralState;
import org.firstinspires.ftc.teamcode.commands.state.TransferState;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class TransferSequence extends SequentialCommandGroup {
    public TransferSequence() {
        super(
                new DepositClawCommand(Deposit.ClawState.CLOSED),
                new WaitCommand(250),
                new IntakeClawCommand(Intake.ClawState.OPEN),
                new WaitCommand(250)
        );
    }
}
