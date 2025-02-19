package org.firstinspires.ftc.teamcode.tests.system;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;

@TeleOp(name = "Deposit Test", group = "Subsystem")
public class DepositTest extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller1;
    private ElapsedTime timer;

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = false;
        Global.USING_DASHBOARD = true;
        Global.DEBUG = true;

        robot.addSubsystem(new Deposit());
        robot.init(hardwareMap, telemetry);

        Global.setState(Global.State.TRANSFER);

        telemetry.setMsTransmissionInterval(250);

        controller1 = new GamepadEx(gamepad1);

        controller1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> robot.deposit.setState(Deposit.State.INTAKE)));

        controller1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> robot.deposit.setState(Deposit.State.TRANSFER)));

        controller1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> robot.deposit.setState(Deposit.State.SPECIMEN_SCORE)));

        controller1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    if (robot.deposit.getState() == Deposit.State.SAMPLE_SCORE)
                        robot.deposit.setState(Deposit.State.SAMPLE_DROP);
                    else robot.deposit.setState(Deposit.State.SAMPLE_SCORE);
                }));

        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.deposit::toggleClawState));

        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }


    @Override
    public void run() {
        robot.read();
        super.run();
        robot.update();
        robot.write();
        robot.clearBulkCache(Global.Hub.CONTROL_HUB);

        telemetry.addData("deposit state", robot.deposit.getState());
        telemetry.addData("deposit claw", robot.deposit.claw_state);
        telemetry.update();
    }
}