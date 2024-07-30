package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class FirstStackGrabCommand extends SequentialCommandGroup {
    public FirstStackGrabCommand() {
        super(
                new ClawCommand(Intake.ClawSide.LEFT, Intake.ClawState.CLOSED),
                new WaitCommand(1000),
                new WristCommand(Intake.WristState.FOLD),
                new WaitCommand(700),
                new ArmSetStateCommand(Arm.ArmState.FLAT)
        );
    }
}
