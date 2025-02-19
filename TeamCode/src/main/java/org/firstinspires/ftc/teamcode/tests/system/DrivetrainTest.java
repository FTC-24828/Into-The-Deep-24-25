package org.firstinspires.ftc.teamcode.tests.system;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeExtensionSetState;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Extension;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

@TeleOp(name = "Drivetrain Test", group = "Subsystem")
public class DrivetrainTest extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller1;
    private ElapsedTime timer;

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = false;
        Global.USING_DASHBOARD = true;
        Global.DEBUG = true;

        robot.addSubsystem(new Drivetrain());
        robot.init(hardwareMap, telemetry);

        Global.setState(Global.State.TRANSFER);

        telemetry.setMsTransmissionInterval(250);

        controller1 = new GamepadEx(gamepad1);

        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }


    @Override
    public void run() {
       if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread(() -> true);
       }

        robot.read();

        Vector2D input_vector = new Vector2D(controller1.getLeftY(), -controller1.getLeftX(), -robot.getYaw());
        input_vector = input_vector.scale(0.5);

        double zPower = -controller1.getRightX() * 0.5;

        //drivetrain move
        robot.drivetrain.move(input_vector, zPower);

        super.run();
        robot.update();
        robot.write();
        robot.clearBulkCache(Global.Hub.CONTROL_HUB);

        telemetry.update();
    }
}