package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetTargetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristSetIncrement;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class YellowPixelSequence extends SequentialCommandGroup {
    public YellowPixelSequence() {
        super(
                new ArmSetStateCommand(Arm.ArmState.SCORING),
                new ArmSetTargetCommand(Global.TETRIX_MOTOR_TPR)
                        .alongWith(new WristCommand(Intake.WristState.SCORING)),

                 new ArmSetTargetCommand(3.0 * Global.TETRIX_MOTOR_TPR / 2 + 150)
                         .alongWith(new WristSetIncrement(-0.3)),
                new WaitCommand(2000),

                new ClawCommand(Intake.ClawSide.LEFT, Intake.ClawState.OPEN),
                new WaitCommand(400),

                new ClawCommand(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED),

                new ArmSetStateCommand( Arm.ArmState.FLAT)
                        .alongWith(new WristCommand(Intake.WristState.FOLD)),
                new WaitCommand(900)
        );
    }
}
