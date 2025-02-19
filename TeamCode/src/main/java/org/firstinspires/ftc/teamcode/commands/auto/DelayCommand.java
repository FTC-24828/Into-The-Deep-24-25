package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class DelayCommand extends SequentialCommandGroup {
    public DelayCommand(Command c, int delay) {
        super(
                new WaitCommand(delay), c
        );
    }
}
