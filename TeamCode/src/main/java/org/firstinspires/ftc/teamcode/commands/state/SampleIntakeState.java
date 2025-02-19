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

public class SampleIntakeState extends ParallelCommandGroup {
    public SampleIntakeState() {
        super(
                new InstantCommand(() -> Global.lockSlowMode(true)),
                new SetRobotState(Global.State.SAMPLE_INTAKE),
                new IntakeSetState(Intake.State.NEUTRAL),
                new DepositSetState(Deposit.State.TRANSFER),
                new IntakeExtensionSetState(Extension.State.EXTEND),
                new DepositExtensionSetState(Extension.State.RETRACT)
        );
    }
}
