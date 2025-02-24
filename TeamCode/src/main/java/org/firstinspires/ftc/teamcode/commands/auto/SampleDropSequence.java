package org.firstinspires.ftc.teamcode.commands.auto;

import android.graphics.Path;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystem.DepositClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositSetState;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;

public class SampleDropSequence extends ParallelCommandGroup {
    public SampleDropSequence() {
        super(
                new DepositSetState(Deposit.State.SAMPLE_DROP),
                new DepositClawCommand(Deposit.ClawState.OPEN)
        );
    }
}
