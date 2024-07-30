package org.firstinspires.ftc.teamcode.common.hardware.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

public class WActuator {
    public final HashMap<String, HardwareDevice> devices = new HashMap<>();

    private DoubleSupplier voltage;
    public ElapsedTime timer;
    private WRobot robot = WRobot.getInstance();

    private double target_position = 0.0;
    private double prev_target = 0.0;
    private double current_position = 0.0;
    private double offset = 0.0;
    private double power = 0.0;
    private double prev_power = 0.0;

    private Supplier<Object> topic;

    public WActuator(HardwareDevice... d) {
        this.topic = null;
        int id = 0;
        for (HardwareDevice device : d) {
            this.devices.put(device.getDeviceName() + " " + id++, device);
        }
        read();
    }

    public WActuator(Supplier<Object> topic, HardwareDevice... d) {
        this.topic = topic;
        int id = 0;
        for (HardwareDevice device : d) {
            this.devices.put(device.getDeviceName() + " " + id++, device);
        }
        read();
    }

    public void periodic() {
    }

    public void read() {
        if (topic != null) {
            Object value = topic.get();
            if (value instanceof Integer) {
                this.current_position = (int) value + offset;       //CORRECT???
                return;
            } else if (value instanceof Double) {
                this.current_position = (double) value + offset;
                return;
            }
        }

        for (HardwareDevice device : devices.values()) {
            if (device instanceof WAnalogEncoder) {
                this.current_position = ((WAnalogEncoder) device).getPosition() + offset;
                return;
            } else if (device instanceof WEncoder) {
                this.current_position = ((WEncoder) device).getPosition() + offset;
                return;
            }
        }
        this.current_position = 0.0;
    }

    public void write() {
        if (Math.abs(power - prev_power) > 0.01 || Math.abs(target_position - prev_target) > 0.01) {
            for (HardwareDevice device : devices.values()) {
                if (device instanceof DcMotor) {
                    double correction = 1.0;
                    if (voltage != null) correction = 12.0 / voltage.getAsDouble();
                    ((DcMotor) device).setPower(WMath.clamp(power * correction, -1, 1));
                    prev_power = power;
                } else if (device instanceof Servo) {
                    ((Servo) device).setPosition(target_position);
                    prev_target = target_position;
                }
            }
        }
    }

    public void reset() {}

    public void setPower(double power) {
        this.power = power;
    }

    public void setTargetPosition(double target_position) {
        this.target_position = target_position;
    }

    /**
     *
     * @param offset offset current_position by + offset
     * @return
     */
    public WActuator setReadingOffset(double offset) {
        this.offset = offset;
        return this;
    }

    public double getReadingOffset() {
        return offset;
    }

    public double getCurrentPosition() {
        return current_position;
    }

    public WActuator setVoltageSupplier(DoubleSupplier voltage) {
        this.voltage = voltage;
        return this;
    }

}
