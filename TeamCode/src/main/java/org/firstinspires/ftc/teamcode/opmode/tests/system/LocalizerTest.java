package org.firstinspires.ftc.teamcode.opmode.tests.system;

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
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;


@TeleOp(name = "localizer test", group = "Utility")
public class LocalizerTest extends CommandOpMode {
    //initialize and getting the robot instance (singleton)
    private final WRobot robot = WRobot.getInstance();

    private double offset = 0;

    //declare controller variables but not initializing them (currently NULL)
    private GamepadEx controller;

    private ElapsedTime timer;

    //called when the "init" button is pressed
    @Override
    public void initialize() {
        super.reset(); //reset the command scheduler (flushing out old commands from last opmode)

        Global.IS_AUTO = true;
        Global.USING_DASHBOARD = false;
        Global.USING_IMU = true;
        Global.USING_WEBCAM = false;
        //additional global flags eg. USING_IMU, USING_DASHBOARD, DEBUG are placed here
        //if is auto, must declare color

        //initialize robot
        robot.addSubsystem(new Drivetrain(), new Arm(), new Intake());
        robot.init(hardwareMap, telemetry);
        robot.localizer.reset(new Pose());

        //get controller
        controller = new GamepadEx(gamepad1);

        controller.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> robot.localizer.reset(new Pose()))
                        .alongWith( new InstantCommand(robot::resetYaw)));

        //display that initialization is complete
        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }

    //called when the play button is pressed
    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread(() -> true);
        }
        robot.read(); //read values from encodes/sensors
        super.run(); //runs commands scheduled above

        //set the drivetrain's motor speed according to controller stick input
        Vector2D local_vector = new Vector2D(controller.getLeftX(), controller.getLeftY(), 0);
        local_vector.scale(0.4);

        robot.periodic(); //calculations/writing data to actuators

        robot.drivetrain.move(local_vector, controller.getRightX() * 0.4);

        robot.write(); //write power to actuators (setting power to motors/servos)
        robot.clearBulkCache(Global.Hub.BOTH); //clear cache accordingly to get new read() values

        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("Pose", robot.localizer.getPose().toString());
        telemetry.addData("yaw", "%.5f", robot.getYaw());
//        telemetry.addData("yaw offset", robot.imu_offset);
        telemetry.addData("yaw diff", "%.5f", robot.getYaw() - robot.localizer.getPose().z);
        telemetry.addData("d_theta", "%.5f", robot.localizer.d_theta);
        telemetry.addData("delta distance", "%.3f, %.3f, %.3f",
                robot.localizer.d_left, robot.localizer.d_middle, robot.localizer.d_right);
        telemetry.addData("Encoder readings", "%.2f, %.2f, %.2f",
                robot.encoder_readings.get(Sensors.Encoder.POD_LEFT),
                robot.encoder_readings.get(Sensors.Encoder.POD_MIDDLE),
                robot.encoder_readings.get(Sensors.Encoder.POD_RIGHT));
        telemetry.update();
    }

    //reset function, called when the opmode is stopped
    @Override
    public void reset() {
        super.reset(); //flush the command scheduler
        robot.reset();
        Global.resetGlobals();
    }
}

