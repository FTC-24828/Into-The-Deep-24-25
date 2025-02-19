package org.firstinspires.ftc.teamcode.common.hardware;

public class Global {
    public enum Hub {CONTROL_HUB, EXPANSION_HUB, BOTH}
    public enum Side {BLUE, RED}
    public enum State {SAMPLE_INTAKE, SPECIMEN_INTAKE, SPECIMEN_SCORING, SAMPLE_SCORING, NEUTRAL, TRANSFER}
    public enum DriveMode {FIELD, ROBOT}

    public static final int TETRIX_MOTOR_TPR = 1440;
    public static final int GOBILDA_ENCODER_TPR = 2000;
    public static double YAW_OFFSET = 0;

    public static boolean USING_DASHBOARD;
    public static boolean IS_AUTO;
    public static boolean USING_IMU;
    public static boolean USING_WEBCAM;
    public static boolean DEBUG;
    private static boolean SLOW_MODE;
    public static boolean slow_mode_lock;
    public static Side SIDE;
    public static State STATE;

    public static void resetGlobals() {
        USING_DASHBOARD = false; IS_AUTO = false; USING_IMU = false; USING_WEBCAM = false; DEBUG = false; SIDE = null; YAW_OFFSET = 0;
    }

    public static void setState(State state) { STATE = state; }

    public static void lockSlowMode(boolean b) { slow_mode_lock = b; SLOW_MODE = b; }

    public static void setSlowMode(boolean b) {
        if (slow_mode_lock) return;
        SLOW_MODE = b;
    }

    public static boolean SLOW_MODE() { return SLOW_MODE; }
}
