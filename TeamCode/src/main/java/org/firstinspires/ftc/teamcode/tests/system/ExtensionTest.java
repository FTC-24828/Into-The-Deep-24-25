package org.firstinspires.ftc.teamcode.tests.system;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.subsystem.DepositExtensionSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeExtensionSetState;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;

@TeleOp(name = "Extension Test", group = "Subsystem")
public class ExtensionTest extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller1;
    private ElapsedTime timer;

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = false;
        Global.USING_DASHBOARD = true;
        Global.DEBUG = true;

        robot.addSubsystem(new Drivetrain(), new Extension());
        robot.init(hardwareMap, telemetry);

        Global.setState(Global.State.TRANSFER);

        telemetry.setMsTransmissionInterval(250);

        controller1 = new GamepadEx(gamepad1);

        controller1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new IntakeExtensionSetState(Extension.State.RETRACT));

        controller1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new IntakeExtensionSetState(Extension.State.EXTEND));

        controller1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(new DepositExtensionSetState(Extension.State.RETRACT),
                        new DepositExtensionSetState(Extension.State.EXTEND),
                        () -> robot.extension.deposit_state == Extension.State.EXTEND
                                || robot.extension.deposit_state == Extension.State.SPECIMEN));

        controller1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ConditionalCommand(new DepositExtensionSetState(Extension.State.SPECIMEN),
                        new DepositExtensionSetState(Extension.State.RETRACT),
                        () -> robot.extension.deposit_state == Extension.State.EXTEND
                                || robot.extension.deposit_state == Extension.State.RETRACT));

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

        telemetry.addData("intake state", robot.extension.intake_state);
        telemetry.addData("intake power", robot.extension.intake_power);
        telemetry.addData("intake tick", robot.extension.intake_tick);
        telemetry.addData("intake offset", robot.extension.intake_offset);
        telemetry.addData("intake raw reading", robot.intSubscriber(Sensors.INTAKE_ENCODER));
        telemetry.addLine("-------------------------");
        telemetry.addData("deposit state", robot.extension.deposit_state);
        telemetry.addData("deposit power", robot.extension.deposit_power);
        telemetry.addData("deposit tick", robot.extension.deposit_tick);
        telemetry.addData("deposit offset", robot.extension.deposit_offset);
        telemetry.addData("deposit raw reading", robot.intSubscriber(Sensors.DEPOSIT_ENCODER));
        telemetry.update();
    }
}