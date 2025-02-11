package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.auto.MoveCommand;
import org.firstinspires.ftc.teamcode.commands.tele.SpecimenAimSequence;
import org.firstinspires.ftc.teamcode.commands.tele.SpecimenInSequence;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Path;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private final ElapsedTime timer = new ElapsedTime();
    private double end_time = 0;
    private double loop_time = 0.0;

    private boolean imu_lock = false;

    private PurePursuit path_controller;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Global.IS_AUTO = true;
        Global.USING_DASHBOARD = true;
        Global.DEBUG = true;
        Global.USING_IMU = true;
        Global.USING_WEBCAM = false;
        Global.SIDE = Global.Side.RED;

        //initialize robot
        robot.addSubsystem(new Drivetrain(), new Arm(), new Intake());
        robot.init(hardwareMap, telemetry);
        Global.setState(Global.State.INIT);
        robot.intake.setClawState(Intake.ClawState.CLOSED);
        robot.read();

        robot.localizer.reset(new Pose(0, 0, 0));
        robot.arm.setArmOffset(0);

        if (Global.USING_DASHBOARD) telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Path scoring_path1 = new Path(
                new Pose(33.5, 12, 0)
        );

        Path push_path = new Path(
                //1st sample
                new Pose(10, 0, 0),
                new Pose(10, -20, 0),
                new Pose(60, -25, 0),
                new Pose(50, -35, 0),
                new Pose(3, -30, 0),

                //2nd sample
                new Pose(60, -35, 0),
                new Pose(50, -40, 0),
                new Pose(3, -43, 0),

//                //3rd sample
//                new Pose(60, -40, 0),
//                new Pose(50, -48, 0),
//                new Pose(3, -45, 0),

                //specimen pickup
                new Pose(30, -10, Math.PI),
                new Pose(7, -20, Math.PI)
        );

        Path scoring_path2 = new Path(
                new Pose(10, 14, 0),
                new Pose(33.5, 14, 0)
        );

        Path scoring_path3 = new Path(
                new Pose(10, 16, 0),
                new Pose(33.5, 16, 0)
        );

        Path scoring_path4 = new Path(
                new Pose(10, 18, 0),
                new Pose(34, 18, 0)
        );

        Path scoring_path5 = new Path(
                new Pose(10, 20, 0),
                new Pose(34, 20, 0)
        );

        Path intaking_path = new Path(
                new Pose(10, -10, Math.PI * 0.8),
                new Pose(30, -20, Math.PI),
                new Pose(7, -20, Math.PI)
        );

        path_controller = new PurePursuit(10, 0, 0);
        path_controller.add(scoring_path1);
        path_controller.add(push_path);
        path_controller.add(intaking_path);
        path_controller.add(scoring_path2);
        path_controller.add(scoring_path3);
        path_controller.add(scoring_path4);
        path_controller.add(scoring_path5);

        while (!isStarted()) {
            double loop = System.nanoTime();
            telemetry.addLine("Autonomous initializing...");
            telemetry.addData("Frequency", 0);
            telemetry.addData("Heading: ", "%.2f", robot.getYaw());
            telemetry.addData("Goal index", path_controller.last_index);
            telemetry.update();
            loop_time = loop;
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        //pre-load specimen
                        new SpecimenAimSequence(),
                        new MoveCommand(() -> path_controller.calculateGoal(0), 2600),
                        new SpecimenInSequence(),

                        new WaitCommand(400),

                        //push all the samples
                        new MoveCommand(() -> path_controller.calculateGoal(1), 13500),

                        //2nd specimen
                        new WaitCommand(200),
                        new SpecimenAimSequence(),
                        new MoveCommand(() -> path_controller.calculateGoal(3), 2600),
                        new SpecimenInSequence(),

                        //3rd specimen
                        new MoveCommand(() -> path_controller.calculateGoal(2), 2800),
                        new SpecimenAimSequence(),
                        new MoveCommand(() -> path_controller.calculateGoal(4), 2600),
                        new SpecimenInSequence(),

                        //4th specimen
                        new MoveCommand(() -> path_controller.calculateGoal(2), 2800),
                        new SpecimenAimSequence(),
                        new MoveCommand(() -> path_controller.calculateGoal(5), 2600),
                        new SpecimenInSequence(),
//
//                        //5th specimen
//                        new MoveCommand(() -> path_controller.calculateGoal(2)),
//                        new SpecimenAimSequence(),
//                        new MoveCommand(() -> path_controller.calculateGoal(6)),
//                        new SpecimenInSequence(),

                        //observation zone park
                        new MoveCommand(() -> new Pose(2, -30, 0)),

                        new InstantCommand(() -> end_time = timer.seconds())

                )
        );
    }

    @Override
    public void run() {
        if (!imu_lock) {
            imu_lock = true;
            robot.startIMUThread(() -> true);
        }
        robot.read();
        super.run();
        robot.update();
        robot.write();
        robot.clearBulkCache(Global.Hub.CONTROL_HUB);

        double loop = System.nanoTime();
        telemetry.addData("Runtime: ", "%.2f", end_time == 0 ? timer.seconds() : end_time);
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Heading: ", "%.2f", robot.getYaw());
        telemetry.addData("Position", "x=%.2f, y=%.2f, z=%.2f",
                robot.localizer.getPose().x,
                robot.localizer.getPose().y,
                robot.localizer.getPose().z
        );
        telemetry.addData("Goal point", "x=%.2f, y=%.2f, z=%.2f",
                path_controller.goal.x,
                path_controller.goal.y,
                path_controller.goal.z
        );
        telemetry.addData("Goal index", path_controller.last_index);
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
