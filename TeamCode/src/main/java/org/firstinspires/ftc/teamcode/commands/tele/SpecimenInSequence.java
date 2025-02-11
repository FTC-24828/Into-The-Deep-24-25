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

public class SpecimenInSequence extends SequentialCommandGroup {
    public SpecimenInSequence() {
        super (
                new ArmSetState(Arm.ArmState.SPECIMEN_IN),
                new WristCommand(Intake.WristState.MIDDLE),
                new ConditionalCommand(
                        new InstantCommand(() -> Global.setState(Global.State.SPECIMEN_INTAKE))
                                .andThen(new WaitCommand(200))
                                .andThen(new ClawCommand(Intake.ClawState.OPEN)),
                        new InstantCommand(() -> Global.setState(Global.State.SPECIMEN_INTAKE)),
                        () -> Global.STATE == Global.State.SPECIMEN_SCORING)
        );
    }
}
