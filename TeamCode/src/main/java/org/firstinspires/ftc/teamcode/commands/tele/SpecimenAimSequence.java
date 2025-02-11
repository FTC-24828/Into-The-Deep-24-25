package org.firstinspires.ftc.teamcode.commands.tele;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class SpecimenAimSequence extends SequentialCommandGroup {
    public SpecimenAimSequence() {
        super (
                new ClawCommand(Intake.ClawState.CLOSED),
                new WaitCommand(500),
                new InstantCommand(() -> Global.setState(Global.State.SPECIMEN_SCORING)),
                new ArmSetState(Arm.ArmState.SPECIMEN_AIM),
                new WristCommand(Intake.WristState.MIDDLE)
        );
    }
}
