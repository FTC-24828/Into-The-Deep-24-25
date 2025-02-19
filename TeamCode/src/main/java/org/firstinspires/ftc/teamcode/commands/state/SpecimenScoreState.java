package org.firstinspires.ftc.teamcode.commands.state;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystem.DepositExtensionSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeExtensionSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSetState;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class SpecimenScoreState extends ParallelCommandGroup {
    public SpecimenScoreState() {
        super(
                new InstantCommand(() -> Global.lockSlowMode(false)),
                new SetRobotState(Global.State.SPECIMEN_SCORING),
                new IntakeSetState(Intake.State.TRANSFER),
                new DepositSetState(Deposit.State.SPECIMEN_SCORE),
                new IntakeExtensionSetState(Extension.State.RETRACT),
                new DepositExtensionSetState(Extension.State.SPECIMEN)
        );
    }
}
