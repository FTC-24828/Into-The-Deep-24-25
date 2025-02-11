package org.firstinspires.ftc.teamcode.commands.tele;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class IntakeBackSequence extends SequentialCommandGroup {
    public IntakeBackSequence() {
        super(
                new InstantCommand(() -> Global.setState(Global.State.INTAKE_BACK)),
                new ArmSetState(Arm.ArmState.BACK_AIM),
                new WristCommand(Intake.WristState.UP)
        );
    }
}
