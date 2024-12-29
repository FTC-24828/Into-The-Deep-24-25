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

import java.util.function.Supplier;

@Config
public class MoveCommand extends CommandBase {
    private final WRobot robot = WRobot.getInstance();

    private final Drivetrain drivetrain = robot.drivetrain;
    private final Localizer localizer = robot.localizer;
    private Supplier<Pose> target_supplier;
    private Pose target_pose;

    public static double tP = 0.5;
    public static double tD = 0.2;

    public static double zP = 5.0;
    public static double zD = 0.25;

    public double TRANSLATIONAL_TOLERANCE = 0.5;
    public double YAW_TOLERANCE = 0.02;

    public static PIDF xController = new PIDF(tP, 0.0, tD);
    public static PIDF yController = new PIDF(tP, 0.0, tD);
    public static PIDF zController = new PIDF(zP, 0.0, zD);

    private ElapsedTime timer;
    private ElapsedTime stable;

    private double WAIT_MS;
    public static double STABLE_MS = 200;

    public MoveCommand(Supplier<Pose> pose) {
        target_supplier = pose;
        WAIT_MS = 10000;

        xController.reset();
        yController.reset();
        zController.reset();
    }

    public MoveCommand(Supplier<Pose> pose, double ms_timeout) {
        this(pose);
        WAIT_MS = ms_timeout;
    }

    public MoveCommand(Pose pose) {
        target_pose = pose;
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
        if (target_supplier != null) target_pose = target_supplier.get();

        Pose delta = target_pose.subtract(robot_pose);

        Vector2D translation_vector = new Vector2D(delta.x, delta.y, robot_pose.z);
        if (translation_vector.magnitude() > 1) translation_vector.normalize();

        double xPower = xController.calculate(translation_vector.x);
        double yPower = yController.calculate(translation_vector.y);
        double zPower = zController.calculate(WMath.wrapAngle(-delta.z));
        xPower = WMath.clamp(xPower, -0.5, 0.5);
        yPower = WMath.clamp(yPower, -0.5, 0.5);
        zPower = WMath.clamp(zPower, -0.5, 0.5);

        return new Pose(xPower, yPower, zPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.move(new Pose());
    }
}
