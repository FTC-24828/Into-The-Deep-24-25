package org.firstinspires.ftc.teamcode.tests.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.autocommand.MoveCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Spline;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Path;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "pathing test", group = "Test")
public class PathingTest extends CommandOpMode{
    private final WRobot robot = WRobot.getInstance();

    private final ElapsedTime timer = new ElapsedTime();
    private double end_time = 0;

    private Object imu_started;

    //called when the "init" button is pressed
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); //flush the command scheduler

        Global.IS_AUTO = true;
        Global.SIDE = Global.Side.RED;
        Global.USING_IMU = true;
        Global.DEBUG = true;
        Global.USING_DASHBOARD = true;

        robot.addSubsystem(new Drivetrain());
        robot.init(hardwareMap, telemetry);

        if (Global.USING_DASHBOARD) telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.localizer.setThetaOffset(0); //OFFSET STARTING VALUE AS NEEDED
        robot.read();

        Path p = new Path(
                new Spline(new Pose(1, 0, 0),
                        new Pose(20, 30, Math.PI * 1.5),
                        new Vector2D(4.5, -10),
                        new Vector2D(6, 25)).generate(8, Spline.EasingType.LINEAR)
        );

        p.add(new Pose(0, 0, Math.PI/2),
                new Pose(10, 10, 0));

        PurePursuit path_controller = new PurePursuit(5, 0, 0);
        path_controller.add(p);

        while (!isStarted()) {
            telemetry.addLine("Autonomous initializing...");
            telemetry.addData("tP", MoveCommand.tP);
            telemetry.addData("tD", MoveCommand.tD);
            telemetry.addData("x error", MoveCommand.xController.last_error);
            telemetry.addData("y error", MoveCommand.yController.last_error);
            telemetry.addData("z error", MoveCommand.zController.last_error);
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        //translationo test
//                        new MoveCommand(new Pose(50, 10, 0), 15000).setTolerance(0.5, 10),
//                        new MoveCommand(new Pose(0, 0, 0), 15000).setTolerance(0.5, 10),

                        //rotation test
                        new MoveCommand(new Pose(0, 0, -Math.PI/1.2), 5000).setTolerance(10, 0.02),
                        new MoveCommand(new Pose(0, 0, 0), 5000).setTolerance(10, 0.02),

//                        new MoveCommand(path_controller.calculateGoal(0)),


                        new InstantCommand(() -> end_time = timer.seconds())

                )
        );
    }

    //called when the play button is pressed
    @Override
    public void run() {
        if (imu_started == null) {
            imu_started = new Object();
            robot.startIMUThread(() -> true);
        }

        robot.read(); //read values from encodes/sensors
        super.run(); //runs commands scheduled in initialize()

        robot.update(); //calculations/writing data to actuators

        robot.write(); //write power to actuators (setting power to motors/servos)
        robot.clearBulkCache(Global.Hub.CONTROL_HUB); //clear cache accordingly to get new read() values

        //display data
        telemetry.addData("Runtime: ", "%.2f", end_time == 0 ? timer.seconds() : end_time);
        telemetry.addData("Current Pose", robot.localizer.getPose().toString());
        telemetry.addData("x error", MoveCommand.xController.last_error);
        telemetry.addData("y error", MoveCommand.yController.last_error);
        telemetry.addData("z error", MoveCommand.zController.last_error);
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

