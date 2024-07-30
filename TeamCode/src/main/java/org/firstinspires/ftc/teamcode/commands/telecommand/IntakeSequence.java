package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class IntakeSequence extends SequentialCommandGroup {
    public IntakeSequence() {
        super(
            new InstantCommand(() -> Global.setState(Global.State.INTAKE)),
            new ArmSetStateCommand(Arm.ArmState.FLAT),
            new WristCommand(Intake.WristState.FLAT),
            new WaitCommand(100),
            new ClawCommand(Intake.ClawSide.BOTH, Intake.ClawState.OPEN)
        );
    }
}
