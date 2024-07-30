package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.drone.DroneResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.drone.LaunchDroneCommand;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Drone;

public class DroneLaunchCommand extends SequentialCommandGroup {
    public DroneLaunchCommand() {
        super(
                new LaunchDroneCommand(),
                new WaitCommand(2000),
                new DroneResetCommand()
        );
    }
}
