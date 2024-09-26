package org.firstinspires.ftc.teamcode.common.hardware.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.common.util.WMath;

public class WAnalogEncoder implements HardwareDevice {
    public AnalogInput encoder;
    private double max_voltage = 3.3;
    private double prev_reading = 0.0;
    public double current_voltage = 0.0;
    private double current_position;
    private boolean inverted = false;
    public double offset = 0.0;

    public WAnalogEncoder(AnalogInput enc) {
        encoder = enc;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    /**returns the current position of the analog encoder in radians*/
    public double getPosition() {
        getVoltage();
        double current_reading = ((!inverted ? current_voltage : max_voltage - current_voltage) + offset)
                / max_voltage * WMath.twoPI;
        double delta = current_reading - prev_reading;
        if (delta > Math.PI) delta -= WMath.twoPI;
        else if (delta < -Math.PI) delta += WMath.twoPI;
        current_position += delta;
        prev_reading = current_reading;
        return current_position;
    }

    public void setVoltageRange(double v) { max_voltage = v; }

    public void setOffset(double o) { offset = o; }

    public void getVoltage() {current_voltage = encoder.getVoltage();}

    public void setInverted(boolean bool) { inverted = bool; }
}
