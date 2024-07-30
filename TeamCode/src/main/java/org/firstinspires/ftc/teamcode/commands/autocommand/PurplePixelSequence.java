package org.firstinspires.ftc.teamcode.commands.autocommand;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class PurplePixelSequence extends SequentialCommandGroup {
    public PurplePixelSequence() {
        super (
                new WristCommand(Intake.WristState.FLAT),
                new WaitCommand(400),
                new ClawCommand(Intake.ClawSide.RIGHT, Intake.ClawState.OPEN),
                new WaitCommand(100),
                new WristCommand(Intake.WristState.FOLD),
                new WaitCommand(200)
            );
    }
}
