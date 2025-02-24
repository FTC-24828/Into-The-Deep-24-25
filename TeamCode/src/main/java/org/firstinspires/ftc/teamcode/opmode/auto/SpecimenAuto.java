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
import org.firstinspires.ftc.teamcode.commands.subsystem.DepositSetState;
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
        robot.localizer.setThetaOffset(0); //OFFSET STARTING VALUE AS NEEDED
        robot.drivetrain.setPodsHeading(Math.tan(20.0/36));

        robot.localizer.reset(new Pose(0, 0, 0));

        if (Global.USING_DASHBOARD) telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Path scoring_path1 = new Path(
                new Pose(35, 20, 0)
        );

        Path scoring_path2 = new Path(
                new Pose(35, 16, 0)
        );

        Path scoring_path3 = new Path(
                new Pose(35, 18, 0)
        );

        Path scoring_path4 = new Path(
                new Pose(35, 20, 0)
        );

        Path scoring_path5 = new Path(
                new Pose(35, 22, 0)
        );

        Path intaking_path = new Path(
                new Pose(25, -20, 0),
                new Pose(-0.5, -20, 0)
        );

        Path push_path = new Path(
                //1st sample
                new Pose(20, 0, 0),
                new Pose(20, -15, 0),
                new Pose(50, -25, 0),
                new Pose(50, -35, 0),
                new Pose(9, -30, 0),

                //2nd sample
                new Pose(50, -35, 0),
                new Pose(50, -43, 0),
                new Pose(9, -43, 0),

                //3rd sample
                new Pose(50, -43, 0),
                new Pose(53, -47, 0),
                new Pose(6, -47 , 0),

                //specimen pickup
                new Pose(25, -20, 0),
                new Pose(-0.5, -20, 0)
        );

        path_controller = new PurePursuit(10, 0, 0);
        path_controller.add(intaking_path); //0
        path_controller.add(scoring_path1); //1
        path_controller.add(scoring_path2); //2
        path_controller.add(scoring_path3); //3
        path_controller.add(scoring_path4); //4
        path_controller.add(scoring_path5); //5
        path_controller.add(push_path);     //6

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
                        new MoveCommand(() -> path_controller.calculateGoal(1), 1700)
                                .alongWith(new DelayCommand(new SpecimenScoreState(), 800)),
                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
                        new WaitCommand(400),
                        new DepositClawCommand(Deposit.ClawState.OPEN),
                        new SpecimenIntakeState(),
                        new WaitCommand(100),

                        //push samples
                        new MoveCommand(() -> path_controller.calculateGoal(6), 12000),

                        //2nd specimen
                        new DepositClawCommand(Deposit.ClawState.CLOSED),
                        new WaitCommand(200),
                        new DepositSetState(Deposit.State.SPECIMEN_SCORE),
                        new MoveCommand(() -> path_controller.calculateGoal(2), 1700)
                                .alongWith(new DelayCommand(new SpecimenScoreState(), 800)),
                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
                        new WaitCommand(500),
                        new DepositClawCommand(Deposit.ClawState.OPEN),
                        new SpecimenIntakeState(),
                        new WaitCommand(100),
                        new MoveCommand(() -> path_controller.calculateGoal(0), 1700),

                        //3rd specimen
                        new DepositClawCommand(Deposit.ClawState.CLOSED),
                        new WaitCommand(200),
                        new DepositSetState(Deposit.State.SPECIMEN_SCORE),
                        new MoveCommand(() -> path_controller.calculateGoal(3), 1700)
                                .alongWith(new DelayCommand(new SpecimenScoreState(), 800)),
                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
                        new WaitCommand(500),
                        new DepositClawCommand(Deposit.ClawState.OPEN),
                        new SpecimenIntakeState(),
                        new WaitCommand(100),
                        new MoveCommand(() -> path_controller.calculateGoal(0), 1700),

                        //4th specimen
                        new DepositClawCommand(Deposit.ClawState.CLOSED),
                        new WaitCommand(200),
                        new DepositSetState(Deposit.State.SPECIMEN_SCORE),
                        new MoveCommand(() -> path_controller.calculateGoal(4), 1700)
                                .alongWith(new DelayCommand(new SpecimenScoreState(), 800)),
                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
                        new WaitCommand(500),
                        new DepositClawCommand(Deposit.ClawState.OPEN),
                        new SpecimenIntakeState(),
                        new WaitCommand(100),
                        new MoveCommand(() -> path_controller.calculateGoal(0), 1700),

                        //5th specimen
                        new DepositClawCommand(Deposit.ClawState.CLOSED),
                        new WaitCommand(200),
                        new DepositSetState(Deposit.State.SPECIMEN_SCORE),
                        new MoveCommand(() -> path_controller.calculateGoal(5), 1700)
                                .alongWith(new DelayCommand(new SpecimenScoreState(), 800)),
                        new DepositExtensionSetState(Extension.State.SPECIMEN_CLIP),
                        new WaitCommand(500),
                        new DepositClawCommand(Deposit.ClawState.OPEN),
                        new SpecimenIntakeState(),
                        new WaitCommand(100),

                        //observation zone park
                        new MoveCommand(new Pose(2, -40, 0)),


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
