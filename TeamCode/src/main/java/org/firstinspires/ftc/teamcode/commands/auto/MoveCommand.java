package org.firstinspires.ftc.teamcode.commands.auto;

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

import java.util.function.Supplier;

@Config
public class MoveCommand extends CommandBase {
    private final WRobot robot = WRobot.getInstance();

    private final Drivetrain drivetrain = robot.drivetrain;
    private final Localizer localizer = robot.localizer;
    private Supplier<Pose> target_supplier;
    public static Pose target_pose;

    public static double tP = 0.2;
    public static double tD = 0.02;

    public static double zP = 1;
    public static double zD = 0.1;

    public double TRANSLATIONAL_TOLERANCE = 0.5;
    public double HEADING_TOLERANCE = Math.toRadians(2);

    public double MAX_TRANSLATIONAL_POWER = 1;
    public double MAX_HEADING_POWER = 0.5;

    public static PIDF xController = new PIDF(tP, 0.0, tD);
    public static PIDF yController = new PIDF(tP, 0.0, tD);
    public static PIDF zController = new PIDF(zP, 0.0, zD);
    public static Pose powers = new Pose();

    public static double zFeedForward;

    private ElapsedTime timer;
    private ElapsedTime stable;

    private double WAIT_MS;
    public static double STABLE_MS = 500;

    public MoveCommand(Supplier<Pose> pose_supplier) {
        target_supplier = pose_supplier;
        target_pose = new Pose();
        WAIT_MS = 10000;

        xController.reset();
        yController.reset();
        zController.reset();
    }

    public MoveCommand(Supplier<Pose> pose_supplier, double ms_timeout) {
        this(pose_supplier);
        WAIT_MS = ms_timeout;
    }

    public MoveCommand(Pose pose) {
        target_pose = pose;
        target_supplier = null;
        WAIT_MS = 5000;

        xController.reset();
        yController.reset();
        zController.reset();
    }

    public MoveCommand(Pose pose, double ms_timeout) {
        this(pose);
        WAIT_MS = ms_timeout;
    }

    public MoveCommand setTolerance(double in_translational_tolerance, double in_yaw_tolerance) {
        TRANSLATIONAL_TOLERANCE = in_translational_tolerance;
        HEADING_TOLERANCE = in_yaw_tolerance;
        return this;
    }

    @Override
    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        if (target_supplier != null) target_pose = target_supplier.get();
        drivetrain.move(powers = calculatePower(localizer.getPose()));
    }

    @Override
    public boolean isFinished() {
        Pose delta = target_pose.subtract(localizer.getPose());

        if (delta.toVector2D().magnitude() > TRANSLATIONAL_TOLERANCE
                || Math.abs(delta.z) > HEADING_TOLERANCE) {
            stable.reset();
        }

        return timer.milliseconds() > WAIT_MS || stable.milliseconds() > STABLE_MS;
    }

    public Pose calculatePower(Pose robot_pose) {
        double v = 12 / robot.getVoltage();

        Pose delta = target_pose.subtract(robot_pose);
        delta.z = WMath.wrapAngle(delta.z);

        Vector2D local_vector = new Vector2D(delta.x, delta.y, -robot_pose.z);

        double xPower = xController.calculate(local_vector.x);
        double yPower = yController.calculate(local_vector.y);

        double max = WMath.max(Math.abs(xPower), Math.abs(yPower), 1);
        if (max > 1) {
            xPower *= 1.0/max;
            yPower *= 1.0/max;
        }

        if (local_vector.magnitude() <= TRANSLATIONAL_TOLERANCE
                && Math.abs(delta.z) > HEADING_TOLERANCE)
            zFeedForward = 0.2 * Math.signum(delta.z);
        else zFeedForward = 0;
        double zPower = zController.calculate(delta.z) + zFeedForward;

        xPower = WMath.clamp(xPower, -1, 1) * MAX_TRANSLATIONAL_POWER;
        yPower = WMath.clamp(yPower, -1, 1) * MAX_TRANSLATIONAL_POWER;
        zPower = WMath.clamp(zPower, -1, 1) * MAX_HEADING_POWER;

        return new Pose(xPower * v, yPower * v, zPower * v);
    }

    @Override
    public void end(boolean interrupted) { drivetrain.move(new Pose()); }
}