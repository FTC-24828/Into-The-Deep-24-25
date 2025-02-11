package org.firstinspires.ftc.teamcode.commands.tele;

import android.graphics.Path;

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

public class IntakeFrontSequence extends SequentialCommandGroup {
    public IntakeFrontSequence() {
        super(
                new InstantCommand(() -> Global.setState(Global.State.INTAKE_FRONT)),
                new ArmSetState(Arm.ArmState.FRONT),
                new WristCommand(Intake.WristState.DOWN)
        );
    }
}
