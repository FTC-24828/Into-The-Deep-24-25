package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.auto.AutoSampleIntakeState;
import org.firstinspires.ftc.teamcode.commands.auto.AutoSpecimenScoreState;
import org.firstinspires.ftc.teamcode.commands.auto.DelayCommand;
import org.firstinspires.ftc.teamcode.commands.auto.MoveCommand;
import org.firstinspires.ftc.teamcode.commands.auto.SampleDropSequence;
import org.firstinspires.ftc.teamcode.commands.state.SampleIntakeState;
import org.firstinspires.ftc.teamcode.commands.state.SampleScoreState;
import org.firstinspires.ftc.teamcode.commands.state.TransferState;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositExtensionSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSetState;
import org.firstinspires.ftc.teamcode.commands.subsystem.SamplePickUpSequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.TransferSequence;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Path;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

@Autonomous(name = "Sample Auto")
public class SampleAuto extends CommandOpMode {
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
        robot.addSubsystem(new Drivetrain(), new Extension(), new Intake(), new Deposit());
        robot.init(hardwareMap, telemetry);
        super.schedule(
                new AutoSpecimenScoreState(),
                new DepositClawCommand(Deposit.ClawState.CLOSED)
        );
        super.run();
        robot.deposit.update();
        robot.deposit.write();
        robot.localizer.setThetaOffset(0); //OFFSET STARTING VALUE AS NEEDED
        robot.drivetrain.setPodsHeading(0);

        robot.localizer.reset(new Pose(0, 0, 0));

        if (Global.USING_DASHBOARD) telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose bucket_pose = new Pose(5, 30, Math.toRadians(-45));

        Path preload_path = new Path(
            new Pose(14, 0, 0),
            bucket_pose
        );

        Path parking_path = new Path(
        );


        path_controller = new PurePursuit(10, 0, 0);
        path_controller.add(preload_path); //0
        path_controller.add(parking_path);

        while (!isStarted()) {
            robot.read();
            robot.update();
            robot.write();
            robot.clearBulkCache(Global.Hub.CONTROL_HUB);
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

                        //pre-load sample
                        new MoveCommand(() -> path_controller.calculateGoal(0), 2400, 0.5)
                                .alongWith(new SampleScoreState()),
                        new WaitCommand(200),
                        new SampleDropSequence(),

                        //2nd sample
                        new MoveCommand(new Pose(12, 15, 0), 1500, 0.5)
                                .alongWith(new DelayCommand(new AutoSampleIntakeState(0.5), 800)),
                        new DelayCommand(new SamplePickUpSequence(), 500),
                        new DelayCommand(new IntakeClawCommand(Intake.ClawState.CLOSED), 300),
                        new DelayCommand(new IntakeSetState(Intake.State.TRANSFER), 100),
                        new DelayCommand(new TransferState(), 100),
                        new DelayCommand(new TransferSequence(), 1000),
                        new MoveCommand(bucket_pose, 1500, 0.5)
                                .alongWith(new DelayCommand(new DepositExtensionSetState(Extension.State.EXTEND), 200)),
                        new DelayCommand(new SampleScoreState(), 300),
                        new DelayCommand(new SampleDropSequence(), 800),
                        new WaitCommand(250),

                        //3rd sample
                        new MoveCommand(new Pose(12, 25, Math.toRadians(5)), 1500, 0.5)
                                .alongWith(new DelayCommand(new AutoSampleIntakeState(0.5), 800)),
                        new DelayCommand(new SamplePickUpSequence(), 500),
                        new DelayCommand(new IntakeClawCommand(Intake.ClawState.CLOSED), 300),
                        new DelayCommand(new IntakeSetState(Intake.State.TRANSFER), 100),
                        new DelayCommand(new TransferState(), 100),
                        new DelayCommand(new TransferSequence(), 1000),
                        new MoveCommand(bucket_pose, 1500, 0.5)
                                .alongWith(new DelayCommand(new DepositExtensionSetState(Extension.State.EXTEND), 200)),
                        new DelayCommand(new SampleScoreState(), 300),
                        new DelayCommand(new SampleDropSequence(), 800),
                        new WaitCommand(250),


                        //4th sample
                        new MoveCommand(new Pose(35, 10, 0), 2500, 0.5)
                                .alongWith(new DelayCommand(new TransferState(), 1000)),
//                                .alongWith(new DepositExtensionSetState(Extension.State.HANG)),
////                                .alongWith(new DelayCommand(new AutoSampleIntakeState(1), 1500)),
////                        new DelayCommand(new SamplePickUpSequence(), 500),
////                        new DelayCommand(new IntakeClawCommand(Intake.ClawState.CLOSED), 300),
//
//                        //level 1 hang
//                        new MoveCommand(new Pose(50, -20, Math.PI/2)),

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
