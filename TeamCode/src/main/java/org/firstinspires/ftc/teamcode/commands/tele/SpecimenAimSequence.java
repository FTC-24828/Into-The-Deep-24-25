package org.firstinspires.ftc.teamcode.commands.tele;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.state.SpecimenIntakeState;
import org.firstinspires.ftc.teamcode.commands.state.SpecimenScoreState;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositSetState;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;

public class SpecimenAimSequence extends SequentialCommandGroup {
    public SpecimenAimSequence() {
        super(
                new DepositSetState(Deposit.State.SPECIMEN_SCORE),
                new WaitCommand(1000),
                new SpecimenScoreState()
        );
    }
}
