package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Size;

import androidx.annotation.GuardedBy;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Drone;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Hang;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WActuator;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
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
    public DcMotorEx middle_port;
    public WEncoder pod_left;
    public WEncoder pod_middle;
    public WEncoder pod_right;
    public Localizer localizer;

    //arm
    public DcMotorEx lift;
    public WServo wrist0;
    public WServo wrist1;
    public WActuator arm_actuator;
    public WActuator wrist_actuator;
    public WEncoder arm_encoder;

    //intake
    public WServo claw_right;
    public WServo claw_left;

    //drone
    public WServo trigger;

    //hang
    public DcMotorEx hang_left;
    public DcMotorEx hang_right;
    public WEncoder hang_encoder;
    public WActuator hang_actuator;
    public WServo hook_left;
    public WServo hook_right;

    private final Object imu_lock = new Object();
    @GuardedBy("imu_lock")
    private IMU imu;
    public Thread imu_thread;
    public double imu_offset = 0;
    private double yaw = 0;

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
    public Arm arm;
    public Intake intake;
    public Hang hang;
    public Drone drone;

    public  HashMap<Sensors.Encoder, Object> encoder_readings;
    public HashMap<Sensors.Sensor, Object> sensor_readings;

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
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ));
            imu.resetYaw();
        }
//        if (Global.USING_IMU) {
//            synchronized (imu_lock) {
//                imu = hardware_map.get(BNO055IMU.class, "imu");
//                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//                imu.initialize(parameters);
//            }
//            //imu_thread.setDaemon(true);
//            resetYaw();
//        }

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

        encoder_readings = new HashMap<>();

        //drivetrain
        motor[0] = hardware_map.get(DcMotorEx.class, "motorFrontRight");
        motor[1] = hardware_map.get(DcMotorEx.class, "motorRearRight");
        motor[2] = hardware_map.get(DcMotorEx.class, "motorRearLeft");
        motor[3] = hardware_map.get(DcMotorEx.class, "motorFrontLeft");
        middle_port = hardware_map.get(DcMotorEx.class, "podMiddle");
        pod_left = new WEncoder(new MotorEx(hardware_map, "motorFrontRight").encoder);
        pod_middle = new WEncoder(new MotorEx(hardware_map, "podMiddle").encoder);
        pod_right = new WEncoder(new MotorEx(hardware_map, "motorRearRight").encoder);
        encoder_readings.put(Sensors.Encoder.POD_LEFT, 0.0);
        encoder_readings.put(Sensors.Encoder.POD_MIDDLE, 0.0);
        encoder_readings.put(Sensors.Encoder.POD_RIGHT, 0.0);
        drivetrain.init(motor);
        localizer.init();


        //arm
        lift = hardware_map.get(DcMotorEx.class, "lift");
        arm_encoder = new WEncoder(new MotorEx(hardware_map, "lift").encoder);
        encoder_readings.put(Sensors.Encoder.ARM_ENCODER, 0);
        arm_actuator = new WActuator(() -> intSubscriber(Sensors.Encoder.ARM_ENCODER), lift)
                .setReadingOffset(0);
        arm.init(lift);


        //intake
        wrist0 = new WServo(hardware_map.get(Servo.class, "wrist0")).setWritingOffset(0.5);
        wrist1 = new WServo(hardware_map.get(Servo.class, "wrist1")).setWritingOffset(0.5);
        claw_right = new WServo(hardware_map.get(Servo.class, "clawRight"));
        claw_left = new WServo(hardware_map.get(Servo.class, "clawLeft"));
        wrist_actuator = new WActuator(wrist0::getPosition, wrist0, wrist1);
        intake.init(wrist0, wrist1, claw_left, claw_right);

        //endgame subsystems
        if (!Global.IS_AUTO) {
            //drone
            if (drone != null) {
                trigger = new WServo(hardware_map.get(Servo.class, "trigger"));
                drone.init(trigger);
            }

            //hang
            if (hang != null) {
                hang_left = hardware_map.get(DcMotorEx.class, "hang0");
                hang_right = hardware_map.get(DcMotorEx.class, "podMiddle");
                hang_encoder = new WEncoder(new MotorEx(hardware_map, "hang0").encoder);
                encoder_readings.put(Sensors.Encoder.HANG_ENCODER, 0);
                hang_actuator = new WActuator(hang_left, hang_right);
                hook_left = new WServo(hardware_map.get(Servo.class, "hook0"));
                hook_right = new WServo(hardware_map.get(Servo.class, "hook1"));
                hang.init(hang_left, hang_right, hook_left, hook_right);
            }
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

                case "Arm": arm = (Arm) subsystem; break;

                case "Intake": intake = (Intake) subsystem; break;

                case "Hang": hang = (Hang) subsystem; break;

                case "Drone": drone = (Drone) subsystem; break;

                default:
                    throw new ClassCastException("Failed to add subsystem.");
            }
        }
    }


    public void periodic() {
        if (timer.seconds() > 5) {
            timer.reset();
            voltage = hardware_map.voltageSensor.iterator().next().getVoltage();
        }

        for (WSubsystem subsystem : subsystems) { subsystem.periodic(); }
    }

    //read encoder values
    public void read () {
        encoder_readings.put(Sensors.Encoder.ARM_ENCODER, arm_encoder.getPosition());
        if (hang != null) encoder_readings.put(Sensors.Encoder.HANG_ENCODER, hang_encoder.getPosition());

        if (Global.IS_AUTO) {
            encoder_readings.put(Sensors.Encoder.POD_LEFT, -pod_left.getPosition());
            encoder_readings.put(Sensors.Encoder.POD_MIDDLE, -pod_middle.getPosition());
            encoder_readings.put(Sensors.Encoder.POD_RIGHT, pod_right.getPosition());
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
        arm = null;
        intake = null;
        hang = null;
        drone = null;
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

//    public void updateYaw() {
//        yaw = WMath.wrapAngle(imu.getAngularOrientation().firstAngle - imu_offset);
//    }

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


    public double doubleSubscriber(Sensors.Encoder topic) {
        Object value = encoder_readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public double doubleSubscriber(Sensors.Sensor topic) {
        Object value = sensor_readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.Encoder topic) {
        Object value = encoder_readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.Sensor topic) {
        Object value = sensor_readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public boolean boolSubscriber(Sensors.Encoder topic) {
        return (boolean) encoder_readings.getOrDefault(topic, false);
    }

    public boolean boolSubscriber(Sensors.Sensor topic) {
        return (boolean) sensor_readings.getOrDefault(topic, false);
    }
}
