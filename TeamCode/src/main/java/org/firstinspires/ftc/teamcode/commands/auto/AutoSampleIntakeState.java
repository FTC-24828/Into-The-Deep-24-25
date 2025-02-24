package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.state.SetRobotState;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositExtensionSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeExtensionSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSetState;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class AutoSampleIntakeState extends ParallelCommandGroup {
    public AutoSampleIntakeState(double d) {
        super(
                new InstantCommand(() -> Global.lockSlowMode(true)),
                new SetRobotState(Global.State.SAMPLE_INTAKE),
                new IntakeSetState(Intake.State.NEUTRAL),
                new DepositSetState(Deposit.State.TRANSFER),
                new IntakeExtensionSetState(Extension.State.EXTEND),
                new DepositExtensionSetState(Extension.State.RETRACT),
                new InstantCommand(() -> WRobot.getInstance().intake_pivot.setPosition(d))
        );
    }
}
