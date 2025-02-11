package org.firstinspires.ftc.teamcode.commands.tele;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class SamplePickUpCommand extends SequentialCommandGroup {
    public SamplePickUpCommand() {
        super(
                new InstantCommand(() -> Global.setState(Global.State.INTAKE_BACK)),
                new ClawCommand(Intake.ClawState.OPEN),
                new WaitCommand(250),
                new ArmSetState(Arm.ArmState.BACK_PICKUP)
        );
    }
}
