package org.firstinspires.ftc.teamcode.tests.system;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.auto.MoveCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Path;

@TeleOp(name = "pathing test", group = "Test")
public class PathingTest extends CommandOpMode{
    private final WRobot robot = WRobot.getInstance();

    private final ElapsedTime timer = new ElapsedTime();
    private double end_time = 0;
    private double loop_time = 0;

    private boolean imu_lock;

    private PurePursuit path_controller;

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
//                new Spline(new Pose(1, 0, 0),
//                        new Pose(20, 30, Math.PI * 1.5),
//                        new Vector2D(4.5, -10),
//                        new Vector2D(6, 25)).generate(8, Spline.EasingType.LINEAR)
        );

        p.add(new Pose(0, 0, 0),
                new Pose(10, 15, 0),
                new Pose(20, 15, 0),
                new Pose(30, -15, 0),
                new Pose(40, -15, 0),
                new Pose(50, 15, 0),
                new Pose(60, 15, 0),
                new Pose(45, -15, 0),
                new Pose(30, -15, 0),
                new Pose(0, 0, 0));

//        p.add(new Pose(),
//                new Pose (70,-10, 0),
//                new Pose (70, 50, 0),
//                new Pose (70, 0, 0),
//                new Pose()
//                );

        path_controller = new PurePursuit(10, 0, 0);
        path_controller.add(p);

        while (!isStarted()) {
            telemetry.addLine("Autonomous initializing...");
            telemetry.addData("x power", MoveCommand.powers.x);
            telemetry.addData("y power", MoveCommand.powers.y);
            telemetry.addData("z power", MoveCommand.powers.z);
            telemetry.addData("x error", MoveCommand.xController.last_error);
            telemetry.addData("y error", MoveCommand.yController.last_error);
            telemetry.addData("z error", MoveCommand.zController.last_error);
            telemetry.addData("z Feedforward", MoveCommand.zFeedForward);
            telemetry.addData("Baseline", 0);
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

//                        //translation test
//                        new MoveCommand(new Pose(50, 10, 0), 7000),
//                        new MoveCommand(new Pose(0, 0, 0), 7000),
//
//                        //rotation test
//                        new MoveCommand(new Pose(0, 0, -Math.PI/2), 5000),
//                        new MoveCommand(new Pose(0, 0, 0), 5000),
//
//                        //combined test
//                        new MoveCommand(new Pose(50, 10, Math.PI/2), 8000),
//                        new MoveCommand(new Pose(0, 0, 0), 8000),

                        //path test
                        new MoveCommand(() -> path_controller.calculateGoal(0)),


                        new InstantCommand(() -> end_time = timer.seconds())

                )
        );
    }

    //called when the play button is pressed
    @Override
    public void run() {
        if (!imu_lock) {
            imu_lock = true;
            robot.startIMUThread(() -> true);
        }

        robot.read(); //read values from encodes/sensors
        super.run(); //runs commands scheduled in initialize()

        robot.update(); //calculations/writing data to actuators

        robot.write(); //write power to actuators (setting power to motors/servos)
        robot.clearBulkCache(Global.Hub.CONTROL_HUB); //clear cache accordingly to get new read() values

        //display data
        double loop = System.nanoTime();
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Runtime: ", "%.2f", end_time == 0 ? timer.seconds() : end_time);
        telemetry.addData("Current Pose", robot.localizer.getPose().toString());

        //Pure pursuit debug
        telemetry.addData("command target", MoveCommand.target_pose);
        telemetry.addData("goal", path_controller.calculateGoal(0));
        telemetry.addData("goal index", path_controller.current_goal);
        telemetry.addData("goal pose", path_controller.path.get(0).get(path_controller.current_goal).toString());
                path_controller.path.get(0).get(path_controller.current_goal);
        telemetry.addData("intersections", path_controller.intersection.size());
        if (!path_controller.intersection.isEmpty())
            telemetry.addData("intersection 1", path_controller.intersection.get(0));

//        path_controller.r = 5;
//        path_controller.findIntersections(new Vector2D(6.1, 5.7), new Vector2D(-3, -6));
//        telemetry.addData("mock up intersections", "x1 = %.2f, y1 = %.2f, x2 = %.2f, y2 = %.2f",
//                path_controller.intersection.get(0).x,
//                path_controller.intersection.get(0).y,
//                path_controller.intersection.get(1).x,
//                path_controller.intersection.get(1).y);

        //PID data
//        telemetry.addData("x power", MoveCommand.powers.x);
//        telemetry.addData("y power", MoveCommand.powers.y);
//        telemetry.addData("z power", MoveCommand.powers.z);
//        telemetry.addData("x error", MoveCommand.xController.last_error);
//        telemetry.addData("y error", MoveCommand.yController.last_error);
//        telemetry.addData("z error", MoveCommand.zController.last_error);
//        telemetry.addData("z Feedforward", MoveCommand.zFeedForward);
//        telemetry.addData("Baseline", 0);
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

