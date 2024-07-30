package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetTargetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristPositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class FirstStackSetup extends SequentialCommandGroup {
    public FirstStackSetup() {
        super (
                new WaitCommand(500),
                new ArmSetStateCommand(Arm.ArmState.SCORING),
                new ArmSetTargetCommand(-50),
                new WristPositionCommand(0.5),

                new WaitCommand(1200),
                new ClawCommand(Intake.ClawSide.LEFT, Intake.ClawState.OPEN)
        );
    }
}
