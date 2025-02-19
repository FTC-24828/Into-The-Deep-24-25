package org.firstinspires.ftc.teamcode.tests.system;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.ToggleClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

@TeleOp(name = "Intake Test", group = "Subsystem")
public class IntakeTest extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller1;
    private ElapsedTime timer;

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = false;
        Global.USING_DASHBOARD = true;
        Global.DEBUG = true;

        robot.addSubsystem(new Intake());
        robot.init(hardwareMap, telemetry);

        Global.setState(Global.State.NEUTRAL);

        telemetry.setMsTransmissionInterval(250);

        controller1 = new GamepadEx(gamepad1);

        controller1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(
                        new IntakeSetState(Intake.State.TRANSFER),
                        new IntakeSetState(Intake.State.NEUTRAL),
                        () -> robot.intake.getState() != Intake.State.TRANSFER
                ));

        controller1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                        new IntakeSetState(Intake.State.INTAKE),
                        new IntakeSetState(Intake.State.NEUTRAL),
                        () -> robot.intake.getState() != Intake.State.INTAKE
                ));

        controller1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new IntakeSetState(Intake.State.PUSH));

        //open/close claw
        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.intake::toggleClawState));

        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }


    @Override
    public void run() {
        robot.read();
        super.run();

        robot.intake.setPivotPosition(0.5
                - controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5
                + controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.5);

        robot.update();
        robot.write();
        robot.clearBulkCache(Global.Hub.CONTROL_HUB);

        telemetry.addData("intake state", robot.intake.getState());
        telemetry.addData("bar state", robot.intake.bar_state);
        telemetry.addData("wrist state", robot.intake.wrist_state);
        telemetry.addData("pivot state", robot.intake.pivot_state);
        telemetry.addData("claw state", robot.intake.claw_state);
        telemetry.addLine("-------------------------");
        telemetry.addData("bar0 pos", robot.bar0.getPosition());
        telemetry.addData("bar1 pos", robot.bar1.getPosition());
        telemetry.addData("four pos", robot.four_bar.target_position);
        telemetry.addData("pivot pos", robot.intake.getPivotPosition());
        telemetry.addData("claw lock", robot.intake.claw_lock);
        telemetry.update();
    }
}