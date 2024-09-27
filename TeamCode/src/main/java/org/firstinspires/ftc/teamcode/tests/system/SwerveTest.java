package org.firstinspires.ftc.teamcode.tests.system;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;

@TeleOp(name = "Swerve test")
public class SwerveTest extends CommandOpMode {
    //initialize and getting the robot instance (singleton)
    private final WRobot robot = WRobot.getInstance();

    //declare controller variables but not initializing them (currently NULL)
    private GamepadEx controller;

    private double loop_time = 0.0;

    int pod_index = 0;
    boolean individual_rotation = false;

    //called when the "init" button is pressed
    @Override
    public void initialize() {
        super.reset(); //reset the command scheduler (flushing out old commands from last opmode)

        Global.IS_AUTO = false;
        Global.DEBUG = true;
        Global.USING_IMU = false;
        //additional global flags eg. USING_IMU, USING_DASHBOARD, DEBUG are placed here
        //if is auto, must declare color

        //initialize robot
        robot.addSubsystem(new Drivetrain());
        robot.init(hardwareMap, telemetry);
//        robot.drivetrain.setPodsHeading(0);
//        robot.drivetrain.overrideHeading();

        controller = new GamepadEx(gamepad1);

        //bind
        Trigger double_joystick = new Trigger(
                (controller.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                        .and(new GamepadButton(controller, GamepadKeys.Button.RIGHT_STICK_BUTTON))::get));

        //manual pods rotation
        controller.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new InstantCommand(() -> robot.drivetrain.rotatePods(0.5)))
                .whenReleased(new InstantCommand(() -> robot.drivetrain.rotatePods(0)));

        controller.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(() -> robot.drivetrain.rotatePods(-0.5)))
                .whenReleased(new InstantCommand(() -> robot.drivetrain.rotatePods(0)));

        controller.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(new InstantCommand(() -> robot.drivetrain.rotateWheels(1)))
                .whenReleased(new InstantCommand(() -> robot.drivetrain.rotateWheels(0)));

        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(() -> robot.drivetrain.rotateWheels(-1)))
                .whenReleased(new InstantCommand(() -> robot.drivetrain.rotateWheels(0)));

        //angle checks
        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenHeld(new InstantCommand(() -> robot.drivetrain.setPodsHeading(0)));
        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenHeld(new InstantCommand(() -> robot.drivetrain.setPodsHeading(-Math.PI / 2)));
        controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenHeld(new InstantCommand(() -> robot.drivetrain.setPodsHeading(Math.PI / 2)));
        controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(new InstantCommand(() -> robot.drivetrain.setPodsHeading(Math.PI)));

        double_joystick.whenActive(() -> robot.drivetrain.reset());

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {if (pod_index < 3) ++pod_index;}));
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {if (pod_index > 0) --pod_index;}));

        //display that initialization is complete
        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }

    //called when the play button is pressed
    @Override
    public void run() {
        robot.read(); //read values from encodes/sensors

        if (gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5) {
            robot.pod[pod_index].setServoPower(gamepad1.right_trigger - gamepad1.left_trigger);
            individual_rotation = true;
        }
        else if (individual_rotation) {
            robot.pod[pod_index].setServoPower(0);
            individual_rotation = false;
        }

        super.run(); //runs commands scheduled in initialize()
        robot.periodic(); //calculations/writing data to actuators
        robot.write(); //write power to actuators (setting power to motors/servos)
        robot.clearBulkCache(Global.Hub.CONTROL_HUB); //clear cache accordingly to get new read() values

        double loop = System.nanoTime();
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addLine("------------------------------------------");
        telemetry.addData("index", pod_index);
        telemetry.addData("heading", Math.toDegrees(robot.drivetrain.current_heading[pod_index]));
        telemetry.addData("target heading", Math.toDegrees(robot.drivetrain.target_heading[pod_index]));
        telemetry.addData("wrap error", robot.pod[pod_index].wrappedError());
        telemetry.addData("servo power", robot.pod[pod_index].getServoPower());
        telemetry.addData("encoder voltage", "%.5fv", robot.heading_encoder[pod_index].getVoltage());
        telemetry.addData("encoder position", Math.toDegrees(robot.heading_encoder[pod_index].getPosition()));
        telemetry.addData("override", robot.pod[pod_index].heading_override);
        telemetry.addData("resetting", robot.pod[pod_index].resetting);
//        telemetry.addData("zeroPowerBehaviour", robot.motor[0].getZeroPowerBehavior());
//        telemetry.addData("motor Power", robot.motor[0].getPower());
//        telemetry.addData("Voltage", "%.2f", robot.getVoltage());
        telemetry.update();
        loop_time = loop;
    }

    //reset function, called when the opmode is stopped
    @Override
    public void reset() {
        super.reset(); //flush the command scheduler
        robot.reset();
        Global.resetGlobals();
    }
}
