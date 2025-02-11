package org.firstinspires.ftc.teamcode.commands.tele;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class InitSequence extends SequentialCommandGroup {
    public InitSequence() {
        super(
                new InstantCommand(() -> Global.setState(Global.State.INIT)),
                new WristCommand(Intake.WristState.UP),
                new ArmSetState(Arm.ArmState.FRONT),
                new ClawCommand(Intake.ClawState.CLOSED)
        );
    }
}
