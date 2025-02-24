package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class SamplePickUpSequence extends SequentialCommandGroup {
    public SamplePickUpSequence() {
        super(
                new IntakeClawCommand(Intake.ClawState.OPEN),
                new WaitCommand(100),
                new IntakeSetState(Intake.State.INTAKE)
        );
    }
}
