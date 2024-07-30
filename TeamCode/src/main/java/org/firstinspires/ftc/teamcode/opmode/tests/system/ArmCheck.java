package org.firstinspires.ftc.teamcode.opmode.tests.system;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmResetPosition;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetTargetCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

@TeleOp(name = "arm check", group = "Utility")
public class ArmCheck extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();
    private DcMotor motor;

    GamepadEx controller;

    @Override
    public void initialize() {
        Global.IS_AUTO = false;

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

        controller = new GamepadEx(gamepad1);

        controller.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ArmSetTargetCommand(0));

        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ArmResetPosition());

        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();
        robot.write();
        robot.clearBulkCache(Global.Hub.BOTH);
    }
}
