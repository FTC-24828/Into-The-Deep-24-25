package org.firstinspires.ftc.teamcode.common.hardware;

public class Global {
    public enum Hub {CONTROL_HUB, EXPANSION_HUB, BOTH}
    public enum Side {BLUE, RED}
    public enum PropLocation {LEFT, RIGHT, CENTER}
    public enum State {SCORING, INTERMEDIATE, INTAKE, LAUNCHING, HANGING}

    public static final int TETRIX_MOTOR_TPR = 1440;
    public static final int GOBILDA_ENCODER_TPR = 2000;
    public static double YAW_OFFSET = 0;

    public static boolean USING_DASHBOARD;
    public static boolean IS_AUTO;
    public static boolean USING_IMU;
    public static boolean USING_WEBCAM;
    public static boolean DEBUG;
    public static Side SIDE;
    public static State STATE = State.INTERMEDIATE;

    public static void resetGlobals() {
        USING_DASHBOARD = false; IS_AUTO = false; USING_IMU = false; USING_WEBCAM = false; DEBUG = false; SIDE = null;
        STATE = State.INTERMEDIATE; YAW_OFFSET = 0;
    }

    public static void setState(State state) {
        STATE = state;
    }
}
