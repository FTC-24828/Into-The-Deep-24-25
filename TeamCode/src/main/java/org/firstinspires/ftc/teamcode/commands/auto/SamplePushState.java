package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.state.SampleIntakeState;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSetState;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class SamplePushState extends SequentialCommandGroup {
    public SamplePushState(double d) {
        super(
                new SampleIntakeState(),
                new IntakeSetState(Intake.State.PUSH),
                new InstantCommand(() -> WRobot.getInstance().intake.setPivotPosition(d))
        );
    }
}
