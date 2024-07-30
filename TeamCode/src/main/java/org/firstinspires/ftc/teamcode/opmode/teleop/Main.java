package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.drone.DroneResetCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.ArmAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.DepositSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.DroneLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.DroneLaunchSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.DroneResetSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.HangMotorCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.HangRetractSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.HangSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.IntakeSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.IntermediateSequence;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Drone;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Hang;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

@TeleOp (name = "MainTeleOp")
public class Main extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller1;
    private GamepadEx controller2;

    private double loop_time = 0.0;

    private double INITIAL_YAW = Global.YAW_OFFSET;  //TODO LINK BETWEEN THE TWO PROGRAMS
    private boolean SLOW_MODE = false;

    private ElapsedTime timer;

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = false;
        Global.USING_DASHBOARD = false;
        Global.DEBUG = true;
        Global.USING_IMU = true;
        Global.USING_WEBCAM = false;

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm(), new Drone(), new Hang());
        robot.init(hardwareMap, telemetry);

        if (Global.USING_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            FtcDashboard.getInstance().startCameraStream(robot.pipeline, 0);
        }

        robot.arm.setArmState(Arm.ArmState.FLAT);
        robot.intake.setWristState(Intake.WristState.FOLD);
        robot.intake.setClawState(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED);
        robot.hang.setHookPosition(0);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        //toggle claw states
        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.BOTH));

        controller1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.LEFT));

        controller1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.RIGHT));

        //arm controls
        controller1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(
                        new ConditionalCommand(
                                new IntakeSequence(),
                                new IntermediateSequence(),
                                () -> Global.STATE == Global.State.INTERMEDIATE
                        ),
                        new InstantCommand(),
                        () -> Global.STATE != Global.State.INTAKE
                ));

        controller1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new ConditionalCommand(
                                new DepositSequence(),
                                new IntermediateSequence(),
                                () -> Global.STATE == Global.State.INTERMEDIATE
                        ),
                        new InstantCommand(),
                        () -> Global.STATE != Global.State.SCORING
                ));

        //slow mode
        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> SLOW_MODE = true))
                .whenReleased(new InstantCommand(() -> SLOW_MODE = false));

        //drone controls
        controller2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new DroneLaunchCommand(),
                        new InstantCommand(),
                        this::isEndGame
                ));

//        controller2.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new ConditionalCommand(
//                        new DroneLaunchCommand(),
//                        new InstantCommand(),
//                        this::isEndGame
//                ));

        controller2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new DroneResetCommand());

        controller2.getGamepadButton(GamepadKeys.Button.B)
                        .whenPressed(new ConditionalCommand(
                                new DroneLaunchCommand(),
                                new InstantCommand(),
                                () -> Global.STATE == Global.State.LAUNCHING && isEndGame()
                        ));

        //hook controls
        controller2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ConditionalCommand(
                        new HangSequence(),
                        new InstantCommand(),
                        this::isEndGame
                ));

        controller2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ConditionalCommand(
                        new HangRetractSequence(),
                        new InstantCommand(),
                        () -> isEndGame() && Global.STATE == Global.State.HANGING
                ));

        //yaw manual reset methods
        controller1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> INITIAL_YAW = robot.getYaw()));

        controller1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> INITIAL_YAW = robot.getYaw() + Math.PI / 2));

        controller1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> INITIAL_YAW = robot.getYaw() - Math.PI / 2));

        controller1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> INITIAL_YAW = robot.getYaw() - Math.PI));

        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }


    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread(() -> true);
        }
        robot.read();

        Vector2D input_vector = new Vector2D(controller1.getLeftX(), controller1.getLeftY(),
                WMath.wrapAngle(robot.getYaw() - INITIAL_YAW));
        if (SLOW_MODE) input_vector.scale(0.4);

        //left trigger gets precedent
        if (controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            super.schedule(new ArmAdjustCommand(SLOW_MODE ? -3 : -5));
        }
        else if (controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            super.schedule(new ArmAdjustCommand(SLOW_MODE ? 3 : 5));
        }

        //hang controls
        if (controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 && isEndGame()) {
            super.schedule(new HangMotorCommand(1));
        }
        else if (controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2 && isEndGame()) {
            super.schedule(new HangMotorCommand(-1));
        }
        else {
            super.schedule(new HangMotorCommand(0));
        }

        super.run();
        robot.periodic();

        robot.drivetrain.move(input_vector, controller1.getRightX() * (SLOW_MODE ? 0.4 : 1));
        robot.write();
        robot.clearBulkCache(Global.Hub.CONTROL_HUB);

        double loop = System.nanoTime();
        telemetry.addData("Timer", "%.0f", timer.seconds());
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Voltage", "%.2f", robot.getVoltage());
        telemetry.addData("Yaw", "%.2f", WMath.wrapAngle(robot.getYaw() - INITIAL_YAW));
        telemetry.addData("State", Global.STATE);

//        telemetry.addData("dev", robot.hang_actuator.devices.toString());

        if (Global.DEBUG) {
            telemetry.addLine("---------------------------");
            telemetry.addData("arm target", robot.arm.target_position);
            telemetry.addData("arm power", robot.arm.power);
            telemetry.addData("arm state", robot.arm.getArmState());
            telemetry.addData("arm angle", "%.2f", Math.toDegrees(robot.arm.arm_angle.getAsDouble()));
            telemetry.addData("arm actuator reading", "%.2f", robot.arm_actuator.getCurrentPosition());
            telemetry.addData("arm encoder reading", "%.2f", robot.encoder_readings.get(Sensors.Encoder.ARM_ENCODER));

            telemetry.addLine("---------------------------");
            telemetry.addData("wrist position", robot.wrist_actuator.getCurrentPosition());
            telemetry.addData("wrist target", robot.intake.target_position);
            telemetry.addData("arm target angle", robot.intake.arm_target_angle);
            telemetry.addData("wrist angle", "%.2f", Math.toDegrees(robot.intake.wrist_angle.getAsDouble()));

            telemetry.addLine("---------------------------");
            telemetry.addData("right claw", "%.2f", robot.claw_right.getPosition());
            telemetry.addData("left claw", "%.2f", robot.claw_left.getPosition());

            telemetry.addLine("---------------------------");
            telemetry.addData("hang state", robot.hang.hang_state);
            telemetry.addData("hang power", robot.hang.power);
//            telemetry.addData("hook position", robot.hook.getPosition());

            telemetry.addLine("---------------------------");
            telemetry.addData("drone state", robot.drone.drone_state);
            telemetry.addData("drone state", robot.trigger.getPosition());
        }

        telemetry.update();
        loop_time = loop;
    }

    @Override
    public void reset() {
        super.reset();
        robot.reset();
        Global.resetGlobals();
    }

    public boolean isEndGame() {
        return timer.seconds() > 0;
    }
}