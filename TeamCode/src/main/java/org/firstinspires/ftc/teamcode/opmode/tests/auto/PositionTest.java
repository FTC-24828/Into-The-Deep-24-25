package org.firstinspires.ftc.teamcode.opmode.tests.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.autocommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

@TeleOp(name = "position test", group = "Utility")
public class PositionTest extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private final ElapsedTime timer = new ElapsedTime();
    private double end_time = 0;

    //called when the "init" button is pressed
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); //flush the command scheduler

        Global.IS_AUTO = true;
        Global.SIDE = Global.Side.RED;
        //additional global flags eg. USING_IMU, USING_DASHBOARD, DEBUG are placed here
        //if is auto, must declare color

        //initialize robot
        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

        //if using ftc dashboard
        if (Global.USING_DASHBOARD) telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.read(); // read encoders/sensors values
        robot.localizer.reset(new Pose(0, 0, 0));

        while (!isStarted()) {
            telemetry.addLine("Autonomous initializing...");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new PositionCommand(new Pose(20, 20, Math.PI / 2)),

                        new InstantCommand(() -> end_time = timer.seconds())

                )
        );
    }

    //called when the play button is pressed
    @Override
    public void run() {
        robot.read(); //read values from encodes/sensors
        super.run(); //runs commands scheduled in initialize()

        robot.periodic(); //calculations/writing data to actuators

        robot.write(); //write power to actuators (setting power to motors/servos)
        robot.clearBulkCache(Global.Hub.BOTH); //clear cache accordingly to get new read() values

        //display data
        telemetry.addData("Runtime: ", end_time == 0 ? timer.seconds() : end_time);
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

