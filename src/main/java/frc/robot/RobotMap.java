package frc.robot;

public class RobotMap {
    public static final String SWERVE_CANBUS_TYPE = "CANivore";
    
    // TO DO : Add CAN IDs

    public static final class LedMap{
        public final static int LED_PWM_PORT = 2;
    }
    public static final class SwerveMap{
        public final static int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public final static int BACK_LEFT_DRIVE_MOTOR_ID = 3;
        public final static int BACK_RIGHT_DRIVE_MOTOR_ID = 5;
        public final static int FRONT_RIGHT_DRIVE_MOTOR_ID = 7;

        public final static int FRONT_LEFT_ANGLE_MOTOR_ID = 2;
        public final static int BACK_LEFT_ANGLE_MOTOR_ID = 4;
        public final static int BACK_RIGHT_ANGLE_MOTOR_ID = 6;
        public final static int FRONT_RIGHT_ANGLE_MOTOR_ID = 8;

        public final static int FRONT_LEFT_CANCODER_MOTOR_ID = 1;
        public final static int BACK_LEFT_CANCODER_MOTOR_ID = 2;
        public final static int BACK_RIGHT_CANCODER_MOTOR_ID = 3;
        public final static int FRONT_RIGHT_CANCODER_MOTOR_ID = 4;
    }


    public static class ShootMap{
        public final static int LEFT_SHOOTER = 10; // Falcon 500
        public final static int RIGHT_SHOOTER = 9; // Falcon 500
    }
    public static class CollectMap{
        public final static int COLLECT_MOTOR_ID = 4; // Neo 550
        public final static int LASER_DIGITAL_INPUT = 0;
        public final static int LASER_DIGITAL_OUTPUT = 1;
    }

    public static final class ArmMap{
        public final static int ARM_LEFT_MOTOR_ID = 1; // Neo
        public final static int ARM_RIGHT_MOTOR_ID = 2; // Neo
        public final static int WRIST_MOTOR_ID = 3; // Vortex

    }
    public static final class HangMap{
        public static final int HANG_LEFT_MOTOR = 5; // Vortex
        public static final int HANG_RIGHT_MOTOR = 6; // Vortex
    }
    public static final class Vision{
        public final static String LIMELIGHT_CENTER_NAME = "limelight"; // Object Detection

    }

 // Using NEO and NEO 550.
}
