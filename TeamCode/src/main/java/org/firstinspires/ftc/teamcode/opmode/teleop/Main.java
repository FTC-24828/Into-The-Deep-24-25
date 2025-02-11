package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmResetCommand;
import org.firstinspires.ftc.teamcode.commands.tele.InitSequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ToggleClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristCommand;
import org.firstinspires.ftc.teamcode.commands.tele.IntakeBackSequence;
import org.firstinspires.ftc.teamcode.commands.tele.IntakeFrontSequence;
import org.firstinspires.ftc.teamcode.commands.tele.SampleDepositSequence;
import org.firstinspires.ftc.teamcode.commands.tele.SampleOutSequence;
import org.firstinspires.ftc.teamcode.commands.tele.SamplePickUpCommand;
import org.firstinspires.ftc.teamcode.commands.tele.SpecimenAimSequence;
import org.firstinspires.ftc.teamcode.commands.tele.SpecimenInSequence;
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
        Global.USING_DASHBOARD = true;
        Global.DEBUG = true;
        Global.USING_IMU = true;
        Global.USING_WEBCAM = false;

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

        Global.setState(Global.State.INTAKE_FRONT);
        schedule(new InitSequence());

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

        //slow mode
        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
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

        //open/close claw
        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ToggleClawCommand());

        //intake front
        controller1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(new IntakeFrontSequence(),
                        new ClawCommand(Intake.ClawState.CLOSED),
                        () -> Global.STATE != Global.State.INTAKE_FRONT));

        //intake back
        controller1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(new IntakeBackSequence(),
                        new SamplePickUpCommand(),
                        () -> Global.STATE != Global.State.INTAKE_BACK))
                .whenReleased(new SequentialCommandGroup(
                        new WristCommand(Intake.WristState.UP),
                        new ClawCommand(Intake.ClawState.CLOSED),
                        new WaitCommand(500),
                        new ArmSetState(Arm.ArmState.BACK_AIM)));

        //specimen
        controller1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ConditionalCommand(new SpecimenInSequence(),
                        new SpecimenAimSequence(),
                        () -> Global.STATE != Global.State.SPECIMEN_INTAKE));

        //sample
        controller1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ConditionalCommand(new SampleOutSequence(),
                        new SampleDepositSequence(),
                        () -> Global.STATE != Global.State.SAMPLE_SCORING));

        //arm reset
        controller1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ArmResetCommand())
                .whenReleased(new InstantCommand((robot.arm::resetArmOffset)));

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

        //reset yaw
        if (controller1.gamepad.guide)  INITIAL_YAW = robot.getYaw();

        double yaw = WMath.wrapAngle(robot.getYaw() - INITIAL_YAW);
        Vector2D input_vector = new Vector2D(controller1.getLeftY(), -controller1.getLeftX(),
                (drive_mode == Global.DriveMode.FIELD ? -yaw : 0));
        if (SLOW_MODE) input_vector = input_vector.scale(0.3);

        robot.drivetrain.move(input_vector, -controller1.getRightX() * (SLOW_MODE ? 0.3 : 1));

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
        telemetry.addData("State", Global.STATE);
        telemetry.addData("Yaw", yaw);
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

            telemetry.addData("arm tick", robot.arm.arm_tick);
            telemetry.addData("arm angle", "%.2f", Math.toDegrees(robot.arm.arm_angle));
            telemetry.addData("arm state", robot.arm.state);
            telemetry.addData("arm reached", robot.arm.reached);
            telemetry.addData("arm power", robot.arm.power);
            telemetry.addData("arm error", Arm.arm_pid.last_error);
            telemetry.addData("arm timer", Arm.arm_profile.timer.seconds());
            telemetry.addData("arm profile pos", Arm.arm_profile.position);
            telemetry.addData("arm state", Arm.arm_profile.state);
//            telemetry.addData("buffer size", robot.arm.tick_buffer.capacity);
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