package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;


@TeleOp (name = "MainTeleOp")
public class Main extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller1;
    private GamepadEx controller2;

    private double loop_time = 0.0;

    private double INITIAL_YAW = Global.YAW_OFFSET;
    private Global.DriveMode drive_mode = Global.DriveMode.FIELD;
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

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

        if (Global.USING_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            FtcDashboard.getInstance().startCameraStream(robot.pipeline, 0);
        }
        telemetry.setMsTransmissionInterval(250);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        //binds
        Trigger double_joystick = new Trigger(
                (controller1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .and(new GamepadButton(controller1, GamepadKeys.Button.RIGHT_STICK_BUTTON))::get));

        //reset yaw
//        controller1.getGamepadButton(GamepadKeys.Button.)
//                        .whenPressed(new InstantCommand(() -> INITIAL_YAW = robot.getYaw()));

        //slow mode
        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> SLOW_MODE = true))
                .whenReleased(new InstantCommand(() -> SLOW_MODE = false));

        //slow mode
        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> SLOW_MODE = true))
                .whenReleased(new InstantCommand(() -> SLOW_MODE = false));

        //switch between field-centric and robot-centric
        double_joystick.whenActive(new InstantCommand(() -> {
           if (drive_mode == Global.DriveMode.FIELD) drive_mode = Global.DriveMode.ROBOT;
           else {
               drive_mode = Global.DriveMode.FIELD;
               INITIAL_YAW = robot.getYaw();
           }
        }));

        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new ClawCommand(Intake.ClawState.OPEN))
                .whenReleased(new ClawCommand(Intake.ClawState.CLOSED));

        controller1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new ConditionalCommand(
                                new WristCommand(Intake.WristState.UP),
                                new WristCommand(Intake.WristState.MIDDLE),
                                () -> robot.intake.wrist_state == Intake.WristState.MIDDLE
                        ),
                        new InstantCommand(),
                        () -> robot.intake.wrist_state != Intake.WristState.UP
                ));

        controller1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(
                        new ConditionalCommand(
                                new WristCommand(Intake.WristState.DOWN),
                                new WristCommand(Intake.WristState.MIDDLE),
                                () -> robot.intake.wrist_state == Intake.WristState.MIDDLE
                        ),
                        new InstantCommand(),
                        () -> robot.intake.wrist_state != Intake.WristState.DOWN
                ));

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

        if (controller1.gamepad.guide)  INITIAL_YAW = robot.getYaw();
        double yaw = WMath.wrapAngle(robot.getYaw() - INITIAL_YAW);
        Vector2D input_vector = new Vector2D(controller1.getLeftY(), -controller1.getLeftX(),
                (drive_mode == Global.DriveMode.FIELD ? yaw : 0));
        if (SLOW_MODE) input_vector = input_vector.scale(0.5);
        robot.drivetrain.move(input_vector, controller1.getRightX() * (SLOW_MODE ? 0.5 : 1));

        robot.arm.setTargetPower(controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                - controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        super.run();

        robot.update();
        robot.write();
        robot.clearBulkCache(Global.Hub.CONTROL_HUB);

        double loop = System.nanoTime();
        telemetry.addData("Timer", "%.0f", timer.seconds());
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Voltage", "%.2f", robot.getVoltage());
        telemetry.addData("Drive Mode", drive_mode);

        if (Global.DEBUG) {
            telemetry.addLine("------------------------------------------");
            telemetry.addData("left y", controller1.getLeftY());
            telemetry.addData("left x", controller1.getLeftX());
            telemetry.addData("right x", controller1.getRightX());

            telemetry.addData("inactive time", robot.drivetrain.inactive_timer.seconds());


            telemetry.addData("motor power", "%+.2f, %+.2f, %+.2f, %+.2f", robot.pod[0].getMotorPower(),
                    robot.pod[1].getMotorPower(),
                    robot.pod[2].getMotorPower(),
                    robot.pod[3].getMotorPower());

            telemetry.addData("servo power", "%+.2f, %+.2f, %+.2f, %+.2f", robot.pod[0].getServoPower(),
                    robot.pod[1].getServoPower(),
                    robot.pod[2].getServoPower(),
                    robot.pod[3].getServoPower());

            telemetry.addData("errors", "%+.2f, %+.2f, %+.2f, %+.2f", robot.pod[0].minError() ,
                    robot.pod[1].minError(),
                    robot.pod[2].minError(),
                    robot.pod[3].minError());
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
        return timer.seconds() > 90;
    }
}