package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Size;

import androidx.annotation.GuardedBy;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.SwervePod;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WActuator;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.WMath;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

public class WRobot {
    private static WRobot robot = null;

    //drivetrain
    public DcMotorEx[] motor = new DcMotorEx[4];
    public CRServo[] servo = new CRServo[4];
    public WAnalogEncoder[] heading_encoder = new WAnalogEncoder[4];
    public SwervePod[] pod = new SwervePod[4];

    public WEncoder pod_x;
    public WEncoder pod_y;
    public Localizer localizer;

    //arm
    public DcMotorEx arm0, arm1;
    public WEncoder arm_encoder;
    public WActuator arm_group;

    //intake
    public Servo wrist, claw;

    private final Object imu_lock = new Object();
    @GuardedBy("imu_lock")
    private IMU imu;
    public Thread imu_thread;
    private volatile double yaw = 0;

    public List<LynxModule> hubs;
    public LynxModule control_hub;
    public LynxModule expansion_hub;

    public VisionPortal vision_portal;
    public PropPipeline pipeline;

    private final ElapsedTime timer = new ElapsedTime();
    private double voltage = 12.0;

    private HardwareMap hardware_map;
    private Telemetry telemetry;

    //subsystems
    private List<WSubsystem> subsystems;
    public Drivetrain drivetrain;
    public Intake intake;
    public Arm arm;

    public  HashMap<Sensors, Object> readings;

    //singleton declaration
    public static WRobot getInstance() {
        if (robot == null) robot = new WRobot();
        return robot;
    }


    //mapping and initializing hardware
    public void init(final HardwareMap hmap, Telemetry telemetry) {
        this.hardware_map = hmap;
        this.telemetry = (Global.USING_DASHBOARD) ? new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry()) : telemetry;

        if (Global.USING_IMU) {
            imu = hardware_map.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )
            ));
            imu.resetYaw();
        }

        if (Global.USING_WEBCAM) {
            pipeline = new PropPipeline();
            vision_portal = new VisionPortal.Builder()
                    .setCamera(hardware_map.get(WebcamName.class, "Webcam"))
                    .setCameraResolution(new Size(640, 480))
                    .addProcessors(pipeline)
                    .enableLiveView(Global.DEBUG)
                    .setAutoStopLiveView(true)
                    .build();
        }

        localizer = new Localizer(new Pose());

        readings = new HashMap<>();

        //drivetrain
        if (drivetrain != null) {
            motor[0] = hardware_map.get(DcMotorEx.class, "motor00");    //  [0]_____[3]
            motor[1] = hardware_map.get(DcMotorEx.class, "motor01");    //   |   ^   |
            motor[2] = hardware_map.get(DcMotorEx.class, "motor02");    //   |   |   |
            motor[3] = hardware_map.get(DcMotorEx.class, "motor03");    //  [1]_____[2]

            servo[0] = hardware_map.get(CRServo.class, "servo00");
            servo[1] = hardware_map.get(CRServo.class, "servo01");
            servo[2] = hardware_map.get(CRServo.class, "servo02");
            servo[3] = hardware_map.get(CRServo.class, "servo03");

            heading_encoder[0] = new WAnalogEncoder(hardware_map.get(AnalogInput.class, "analog00"));
            heading_encoder[1] = new WAnalogEncoder(hardware_map.get(AnalogInput.class, "analog01"));
            heading_encoder[2] = new WAnalogEncoder(hardware_map.get(AnalogInput.class, "analog02"));
            heading_encoder[3] = new WAnalogEncoder(hardware_map.get(AnalogInput.class, "analog03"));

            pod[0] = new SwervePod();
            pod[1] = new SwervePod();
            pod[2] = new SwervePod();
            pod[3] = new SwervePod();

            pod_y = new WEncoder(new MotorEx(hardware_map, "motor00").encoder);
            pod_x = new WEncoder(new MotorEx(hardware_map, "motor03").encoder);

            drivetrain.init(motor, servo, heading_encoder);
            localizer.init();
        }

//        intake
        if (intake != null) {
            wrist = hardware_map.get(Servo.class, "servo04");
            claw = hardware_map.get(Servo.class, "servo05");

            intake.init(claw, wrist);
        }

        if (arm != null) {
            arm0 = hardware_map.get(DcMotorEx.class, "motor10");
            arm1 = hardware_map.get(DcMotorEx.class, "motor11");
            arm_encoder = new WEncoder(new MotorEx(hardware_map, "motor01").encoder);

            arm_group = new WActuator(() -> intSubscriber(Sensors.ARM_ENCODER), arm0, arm1);
            readings.put(Sensors.ARM_ENCODER, 0);

            arm.init(arm0, arm1);
        }

        //lynx hubs
        hubs = hardware_map.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (module.isParent() && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber()))
                    control_hub = module;
            else expansion_hub = module;
        }

        voltage = hardware_map.voltageSensor.iterator().next().getVoltage();
    }


    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems = new ArrayList<>();
        this.subsystems.addAll(Arrays.asList(subsystems));
        for (WSubsystem subsystem : subsystems) {
            switch (subsystem.getClass().getSimpleName()) {
                case "Drivetrain": drivetrain = (Drivetrain) subsystem; break;
                case "Intake": intake = (Intake) subsystem; break;
                case "Arm": arm = (Arm) subsystem; break;
                default:
                    throw new ClassCastException("Failed to add subsystem.");
            }
        }
    }


    public void update() {
        if (timer.seconds() > 5) {
            timer.reset();
            voltage = hardware_map.voltageSensor.iterator().next().getVoltage();
        }

        for (WSubsystem subsystem : subsystems) { subsystem.update(); }
    }

    public void read () {
        if (arm != null) readings.put(Sensors.ARM_ENCODER, arm_encoder.getPosition());

        if (Global.IS_AUTO) {
            readings.put(Sensors.POD_X, -pod_x.getPosition());
            readings.put(Sensors.POD_Y, pod_y.getPosition());
            localizer.update();
        }

        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
        drivetrain = null;
    }

    public double getVoltage() {
        return voltage;
    }

    public void startIMUThread(BooleanSupplier predicate) {
        if (Global.USING_IMU) {
            imu_thread = new Thread(() -> {
                while (predicate.getAsBoolean()) {
                    synchronized (imu_lock) {
                        yaw = WMath.wrapAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                    }
                }
            });
            imu_thread.start();
        }
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public double getYaw() {
        return yaw;
    }

    public void clearBulkCache(@NonNull Global.Hub hub) {
        switch (hub) {
            case CONTROL_HUB:
                control_hub.clearBulkCache();
                break;

            case EXPANSION_HUB:
                expansion_hub.clearBulkCache();
                break;

            case BOTH:
                control_hub.clearBulkCache();
                expansion_hub.clearBulkCache();
                break;
        }
    }
    public double doubleSubscriber(Sensors topic) {
        Object value = readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors topic) {
        Object value = readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public boolean boolSubscriber(Sensors topic) {
        return (boolean) readings.getOrDefault(topic, false);
    }
}
