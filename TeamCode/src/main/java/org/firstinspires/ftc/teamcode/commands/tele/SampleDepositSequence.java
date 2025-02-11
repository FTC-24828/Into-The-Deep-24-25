package org.firstinspires.ftc.teamcode.commands.tele;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class SampleDepositSequence extends SequentialCommandGroup {
    public SampleDepositSequence() {
        super(
               new WristCommand(Intake.WristState.UP),
               new WaitCommand(500),
               new ClawCommand(Intake.ClawState.OPEN),
               new WaitCommand(500),
               new WristCommand(Intake.WristState.MIDDLE)
        );
    }
}
