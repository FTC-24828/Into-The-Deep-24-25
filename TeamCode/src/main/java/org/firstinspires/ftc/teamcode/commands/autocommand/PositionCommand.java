package org.firstinspires.ftc.teamcode.commands.autocommand;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.controllers.PIDF;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

@Config
public class PositionCommand extends CommandBase {
    private final WRobot robot = WRobot.getInstance();

    private final Drivetrain drivetrain = robot.drivetrain;
    private final Localizer localizer = robot.localizer;
    private final Pose target_pose;

    public static double xP = 0.2;
    public static double xD = 0.02;

    public static double yP = 0.2;
    public static double yD = 0.02;

    public static double zP = 1.5;
    public static double zD = 0.2;

    public double TRANSLATIONAL_TOLERANCE = 0.5;
    public double YAW_TOLERANCE = 0.01;

    public static PIDF xController = new PIDF(xP, 0.0, xD);
    public static PIDF yController = new PIDF(yP, 0.0, yD);
    public static PIDF zController = new PIDF(zP, 0.0, zD);

    private ElapsedTime timer;
    private ElapsedTime stable;

    private double WAIT_MS;
    public static double STABLE_MS = 500;

    public PositionCommand(Pose pose) {
        target_pose = pose;
        WAIT_MS = 5000;

        xController.reset();
        yController.reset();
        zController.reset();
    }

    public PositionCommand(Pose pose, double ms_timeout) {
        this(pose);
        WAIT_MS = ms_timeout;
    }

    public PositionCommand setTolerance(double in_translational_tolerance, double in_yaw_tolerance) {
        TRANSLATIONAL_TOLERANCE = in_translational_tolerance;
        YAW_TOLERANCE = in_yaw_tolerance;
        return this;
    }

    @Override
    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose robot_pose = localizer.getPose();

        drivetrain.move(calculatePower(robot_pose));
    }

    @Override
    public boolean isFinished() {
        Pose delta = target_pose.subtract(localizer.getPose());

        if (delta.toVector2D().magnitude() > TRANSLATIONAL_TOLERANCE
                || Math.abs(delta.z) > YAW_TOLERANCE) {
            stable.reset();
        }

        return timer.milliseconds() > WAIT_MS || stable.milliseconds() > STABLE_MS;
    }

    public Pose calculatePower(Pose robot_pose) {
        double xPower = xController.calculate(robot_pose.x, target_pose.x);
        double yPower = yController.calculate(robot_pose.y, target_pose.y);
        double zPower = zController.calculate(WMath.wrapAngle(robot_pose.z - target_pose.z));

        Vector2D translation_vector = new Vector2D(xPower, yPower, robot_pose.z).clamp(-0.4, 0.4);
        zPower = WMath.clamp(zPower, -0.6, 0.6); //TODO: tune this

        return new Pose(translation_vector, zPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.move(new Pose());
    }
}
