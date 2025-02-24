package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.state.NeutralState;
import org.firstinspires.ftc.teamcode.commands.state.SampleIntakeState;
import org.firstinspires.ftc.teamcode.commands.state.SampleScoreState;
import org.firstinspires.ftc.teamcode.commands.state.SpecimenIntakeState;
import org.firstinspires.ftc.teamcode.commands.state.SpecimenScoreState;
import org.firstinspires.ftc.teamcode.commands.state.TransferState;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.SamplePickUpSequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.TransferSequence;
import org.firstinspires.ftc.teamcode.commands.tele.SpecimenAimSequence;
import org.firstinspires.ftc.teamcode.common.controllers.PIDF;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

@Photon
@Config
@TeleOp (name = "MainTeleOp")
public class Main extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller1;
    private GamepadEx controller2;

    private double loop_time = 0.0;

    private double INITIAL_YAW = Global.YAW_OFFSET;
    private Global.DriveMode drive_mode = Global.DriveMode.FIELD;

    public static double kP = 1;
    public static double kD = 0.02;
    public static PIDF heading_pid = new PIDF(kP, 0.0, kD, 0.0);

    private ElapsedTime timer;

//    private WLogger logger;

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = false;
        Global.USING_DASHBOARD = true;
        Global.DEBUG = false;
        Global.USING_IMU = true;
        Global.USING_WEBCAM = false;
        Global.setSlowMode(false);

        robot.addSubsystem(new Drivetrain(), new Intake(), new Extension(), new Deposit());
        robot.init(hardwareMap, telemetry);

        Global.setState(Global.State.NEUTRAL);

        if (Global.USING_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            FtcDashboard.getInstance().startCameraStream(robot.pipeline, 0);
        }
        telemetry.setMsTransmissionInterval(250);

//        if (Global.DEBUG) logger = new WLogger();

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        //binds
        Trigger double_joystick = new Trigger(
                (controller1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                        .and(new GamepadButton(controller1, GamepadKeys.Button.RIGHT_STICK_BUTTON))::get));

        //slow mode
        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> Global.setSlowMode(true)))
                .whenReleased(new InstantCommand(() -> Global.setSlowMode(false)));

        //switch between field-centric and robot-centric
        double_joystick.whenActive(new InstantCommand(() -> {
            if (drive_mode == Global.DriveMode.FIELD) drive_mode = Global.DriveMode.ROBOT;
            else {
                drive_mode = Global.DriveMode.FIELD;
                INITIAL_YAW = robot.getYaw();
            }
        }));

        //extend intake
        controller1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new SampleIntakeState(),
                        new SamplePickUpSequence(),
                        () -> Global.STATE != Global.State.SAMPLE_INTAKE
                ))
                .whenReleased(new ConditionalCommand(
                        new IntakeClawCommand(Intake.ClawState.CLOSED)
                                .andThen(new WaitCommand(250)),
                        new InstantCommand(),
                        () -> robot.intake.getState() == Intake.State.INTAKE)
                        .andThen(new SampleIntakeState()));

        //transfer state
        controller1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(
                        new IntakeSetState(Intake.State.TRANSFER).andThen(new WaitCommand(350)),
                        new InstantCommand(),
                        () -> Global.STATE == Global.State.SAMPLE_INTAKE)
                        .andThen(new NeutralState(), new WaitCommand(600), new TransferState()));

        //specimen
        controller1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ConditionalCommand(
                        new SpecimenIntakeState()
                                .andThen(new DepositClawCommand(Deposit.ClawState.OPEN)),
                        new SpecimenAimSequence(),
                        () -> Global.STATE != Global.State.SPECIMEN_INTAKE
                ));

        //sample
        controller1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ConditionalCommand(
                        new TransferSequence().andThen(new SampleScoreState()),
                        new NeutralState(),
                        () -> Global.STATE == Global.State.TRANSFER
                ));

        //RB action
        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                   if (Global.STATE == Global.State.SPECIMEN_SCORING) {
                       robot.extension.setDepositState(Extension.State.SPECIMEN_CLIP);
                       robot.deposit.setClawState(Deposit.ClawState.CLOSED);
                   }
                   else if (Global.STATE == Global.State.SAMPLE_SCORING) {
                       robot.deposit.setState(Deposit.State.SAMPLE_DROP);
                       robot.deposit.setClawState(Deposit.ClawState.OPEN);
                   }
                   else {
                       robot.intake.toggleClawState();
                       robot.deposit.toggleClawState();
                   }
                }))
                .whenReleased(new ConditionalCommand(
                        new SpecimenIntakeState()
                                .andThen(new DepositClawCommand(Deposit.ClawState.OPEN)),
                        new InstantCommand(() -> {
                            if (Global.STATE == Global.State.SAMPLE_SCORING) {
                                robot.deposit.setState(Deposit.State.NEUTRAL);
                                robot.deposit.setClawState(Deposit.ClawState.CLOSED);
                            }}), () -> Global.STATE == Global.State.SPECIMEN_SCORING ));

        controller1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                        .whenPressed(new InstantCommand(() -> robot.extension_motor[1].setPower(-0.5)))
                .whenReleased(new InstantCommand(() -> {
                    robot.extension.deposit_offset = robot.intSubscriber(Sensors.DEPOSIT_ENCODER);
                }));

        //reset yaw
        controller1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> INITIAL_YAW = robot.getYaw()));

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
        if (Global.SLOW_MODE()) input_vector = input_vector.scale(0.3);

        boolean heading_lock = -controller1.getRightY() > 0.5;
        double zPower = heading_lock ? heading_pid.calculate(yaw, 0) :
            -controller1.getRightX() * (Global.SLOW_MODE()? 0.3 : 0.7);

        //drivetrain move
        robot.drivetrain.move(input_vector, zPower);

        //pivot control
        robot.intake.setPivotPosition(0.5
                - controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5
                + controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.5);

        super.run();

        robot.update();
        robot.write();
        robot.clearBulkCache(Global.Hub.CONTROL_HUB);

        double loop = System.nanoTime();
        telemetry.addData("Timer", "%.0f", timer.seconds());
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("State", Global.STATE);
        telemetry.addData("Yaw", yaw);
        telemetry.addData("Drive Mode", drive_mode);

        if (Global.DEBUG) {
            telemetry.addLine("------------------------------------------");
            telemetry.addData("left y", controller1.getLeftY());
            telemetry.addData("left x", controller1.getLeftX());
            telemetry.addData("right x", controller1.getRightX());
            telemetry.addData("right y", controller1.getRightY());

            telemetry.addData("heading lock", heading_lock);

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

            telemetry.addLine("------------------------------------------");
            telemetry.addData("Intake State", robot.intake.getState());
            telemetry.addData("Deposit State", robot.deposit.getState());
            telemetry.addData("Intake Extension State", robot.extension.intake_state);
            telemetry.addData("Deposit Extension State", robot.extension.deposit_state);
            telemetry.addData("Intake Claw ", robot.intake.claw_state);
            telemetry.addData("Intake Claw Lock", robot.intake.claw_lock);
            telemetry.addData("Deposit Claw State", robot.deposit.claw_state);
            telemetry.addData("Deposit Claw Lock", robot.deposit.claw_lock);

            telemetry.addLine("------------------------------------------");
            telemetry.addData("Intake Extension Power", robot.intake_extension.power);
            telemetry.addData("Intake Extension Velocity", robot.motor[1].getVelocity());
            telemetry.addData("Intake Reached", robot.extension.intake_reached);
            telemetry.addData("Intake Extension Tick", robot.extension.intake_tick);
            telemetry.addData("Deposit Extension Power", robot.deposit_extension.power);
            telemetry.addData("Deposit Extension Tick", robot.extension.deposit_tick);
            telemetry.addData("Deposit Reached", robot.extension.deposit_reached);
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