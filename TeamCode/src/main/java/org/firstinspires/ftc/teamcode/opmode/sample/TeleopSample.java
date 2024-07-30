package org.firstinspires.ftc.teamcode.opmode.sample;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

@Disabled //remove this to activate opmode
@TeleOp(name = "tele opmode name")
public class TeleopSample extends CommandOpMode {
    //initialize and getting the robot instance (singleton)
    private final WRobot robot = WRobot.getInstance();

    //declare controller variables but not initializing them (currently NULL)
    private GamepadEx controller;

    //called when the "init" button is pressed
    @Override
    public void initialize() {
        super.reset(); //reset the command scheduler (flushing out old commands from last opmode)

        Global.IS_AUTO = false;
        //additional global flags eg. USING_IMU, USING_DASHBOARD, DEBUG are placed here
        //if is auto, must declare color

        //initialize robot
        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

        //get controller
        controller = new GamepadEx(gamepad1);

        //initialize controller buttons mappings here (during initialize())
        //maps right bumper to a command
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(
                        // -> your command here <-
                ));

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
        super.run(); //runs commands scheduled in initialize()

        //set the drivetrain's motor speed according to controller stick input
        robot.drivetrain.move(controller.getLeftX(), controller.getLeftY(), controller.getRightX());

        if (controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            super.schedule(
                    // command(s) to be run when left trigger is pressed more than halfway
            );
        }
        else if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            super.schedule(
                    // command(s) to be run when left trigger is pressed more than halfway
            );
        }

        robot.periodic(); //calculations/writing data to actuators

        robot.write(); //write power to actuators (setting power to motors/servos)
        robot.clearBulkCache(Global.Hub.BOTH); //clear cache accordingly to get new read() values

        telemetry.addData("Voltage", robot.getVoltage());
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
