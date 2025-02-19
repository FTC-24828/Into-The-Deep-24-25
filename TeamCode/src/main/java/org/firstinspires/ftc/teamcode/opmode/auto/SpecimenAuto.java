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

import org.firstinspires.ftc.teamcode.commands.auto.AutoSpecimenScoreState;
import org.firstinspires.ftc.teamcode.commands.auto.DelayCommand;
import org.firstinspires.ftc.teamcode.commands.auto.MoveCommand;
import org.firstinspires.ftc.teamcode.commands.auto.SamplePushState;
import org.firstinspires.ftc.teamcode.commands.state.NeutralState;
import org.firstinspires.ftc.teamcode.commands.state.SampleIntakeState;
import org.firstinspires.ftc.teamcode.commands.state.SpecimenIntakeState;
import org.firstinspires.ftc.teamcode.commands.state.SpecimenScoreState;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositExtensionSetState;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Path;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;

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
        robot.addSubsystem(new Drivetrain(), new Extension(), new Intake(), new Deposit());
        robot.init(hardwareMap, telemetry);
        super.schedule(
                new AutoSpecimenScoreState(),
                new DepositClawCommand(Deposit.ClawState.CLOSED)
        );
        super.run();
        robot.deposit.update();
        robot.deposit.write();

        robot.localizer.reset(new Pose(0, 0, 0));

        if (Global.USING_DASHBOARD) telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Path scoring_path1 = new Path(
                new Pose(35.5, 12, 0)
        );

        Path scoring_path2 = new Path(
                new Pose(10, 14, 0),
                new Pose(35.5, 14, 0)
        );

        Path scoring_path3 = new Path(
                new Pose(10, 16, 0),
                new Pose(35.5, 16, 0)
        );

        Path scoring_path4 = new Path(
                new Pose(10, 18, 0),
                new Pose(35.5, 18, 0)
        );

        Path scoring_path5 = new Path(
                new Pose(10, 20, 0),
                new Pose(35.5, 20, 0)
        );

        Path intaking_path = new Path(
                new Pose(30, -20, 0),
                new Pose(7, -20, 0)
        );

        path_controller = new PurePursuit(10, 0, 0);
        path_controller.add(intaking_path); //0
        path_controller.add(scoring_path1); //1
        path_controller.add(scoring_path2); //2
        path_controller.add(scoring_path3); //3
        path_controller.add(scoring_path4); //4
        path_controller.add(scoring_path5); //5

        while (!isStarted()) {
            double loop = System.nanoTime();
            telemetry.addLine("Autonomous initializing...");
            telemetry.addData("Frequency", 0);
            telemetry.addData("Heading: ", "%.2f", robot.getYaw());
            telemetry.addData("Goal index", path_controller.last_index);
            telemetry.addData("Deposit state", robot.deposit.getState());
            telemetry.addData("Deposit wrist state", robot.deposit.wrist_state);
            telemetry.update();
            loop_time = loop;
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new AutoSpecimenScoreState(),

                        //pre-load specimen
                        new MoveCommand(() -> path_controller.calculateGoal(1), 2600)
                                .alongWith(new DelayCommand(new SpecimenScoreState(), 500)),
                        new WaitCommand(200),
                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
                        new WaitCommand(300),
                        new DepositClawCommand(Deposit.ClawState.OPEN),
                        new WaitCommand(250),
                        new NeutralState(),

                        //push 1st sample
                        new MoveCommand(() -> new Pose(10, -20, Math.toRadians(-50)), 1500)
                                .alongWith(new DelayCommand(new SamplePushState(1), 800)),
                        new WaitCommand(700),
                        new MoveCommand(() -> new Pose(10, -20, Math.toRadians(-120)), 1500),
                        new SampleIntakeState(),

                        //push 2nd sample
                        new MoveCommand(() -> new Pose(10, -35, Math.toRadians(-50)), 1500)
                                .alongWith(new DelayCommand(new SamplePushState(1), 800)),
                        new WaitCommand(700),
                        new MoveCommand(() -> new Pose(10, -35, Math.toRadians(-120)), 1500),
                        new SampleIntakeState(),

                        //push 3st sample
                        new MoveCommand(() -> new Pose(10, -50, Math.toRadians(-50)), 1500)
                                .alongWith(new DelayCommand(new SamplePushState(1), 800)),
                        new WaitCommand(700),
                        new MoveCommand(() -> new Pose(10, -50, Math.toRadians(-120)), 1500),
                        new SampleIntakeState(),

                        //2nd specimen
                        new SpecimenIntakeState(),
//                        new MoveCommand(() -> path_controller.calculateGoal(0), 2600),
//                        new DepositClawCommand(Deposit.ClawState.CLOSED),
//                        new WaitCommand(250),
//                        new SpecimenScoreState(),
//                        new MoveCommand(() -> path_controller.calculateGoal(2), 2600),
//                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
//                        new WaitCommand(250),
//                        new DepositClawCommand(Deposit.ClawState.OPEN),
//
//                        //3rd specimen
//                        new SpecimenIntakeState(),
//                        new MoveCommand(() -> path_controller.calculateGoal(0), 2600),
//                        new DepositClawCommand(Deposit.ClawState.CLOSED),
//                        new WaitCommand(250),
//                        new SpecimenScoreState(),
//                        new MoveCommand(() -> path_controller.calculateGoal(3), 2600),
//                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
//                        new WaitCommand(250),
//                        new DepositClawCommand(Deposit.ClawState.OPEN),
//
//                        //4th specimen
//                        new SpecimenIntakeState(),
//                        new MoveCommand(() -> path_controller.calculateGoal(0), 2600),
//                        new DepositClawCommand(Deposit.ClawState.CLOSED),
//                        new WaitCommand(250),
//                        new SpecimenScoreState(),
//                        new MoveCommand(() -> path_controller.calculateGoal(4), 2600),
//                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
//                        new WaitCommand(250),
//                        new DepositClawCommand(Deposit.ClawState.OPEN),
//
//                        //5th specimen
//                        new SpecimenIntakeState(),
//                        new MoveCommand(() -> path_controller.calculateGoal(0), 2600),
//                        new DepositClawCommand(Deposit.ClawState.CLOSED),
//                        new WaitCommand(250),
//                        new SpecimenScoreState(),
//                        new MoveCommand(() -> path_controller.calculateGoal(5), 2600),
//                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
//                        new WaitCommand(250),
//                        new DepositClawCommand(Deposit.ClawState.OPEN),

                        //observation zone park
//                        new MoveCommand(() -> new Pose(2, -50, 0)),

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
                MoveCommand.target_pose.x,
                MoveCommand.target_pose.y,
                MoveCommand.target_pose.z
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
