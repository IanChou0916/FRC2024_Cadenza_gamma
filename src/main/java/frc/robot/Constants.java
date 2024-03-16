package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.convert.Conversions;
import static edu.wpi.first.wpilibj.GenericHID.*;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.*;

public class Constants {
    public static boolean NAVX_INVERTED = true;
    public static byte NAVX_UPDATE_RATE = 127;

    public final static double MAX_VOLTAGE = 12.0;

    public final static class JoyStickConstants {
        public static final double DRIVE_JOYSTICK_DEADBAND = 0.1;
        public static final double OPERATOR_JOYSTICK_DEADBAND = 0.05;
        public static final double NOTICE_VALUE = 0.5;
        public static final RumbleType COLLECT_RUMBLE = kLeftRumble;
        public static final RumbleType SHOOT_RUMBLE = kRightRumble;
        public static final RumbleType NORMAL_RUMBLE = kLeftRumble;
    }

    public static final class LEDConstants{
        public final static byte LED_LENGTH = 60;
    }
    public static final class SwerveConstants{
        public final static Rotation2d FRONT_LEFT_ANGLE_OFFSET = Rotation2d.fromRotations(0.405029);
        public final static Rotation2d BACK_LEFT_ANGLE_OFFSET = Rotation2d.fromRotations(0.743896);
        public final static Rotation2d BACK_RIGHT_ANGLE_OFFSET = Rotation2d.fromRotations(0.696777);
        public final static Rotation2d FRONT_RIGHT_ANGLE_OFFSET = Rotation2d.fromRotations(0.054199);

        //
        public static final double SWERVE_CHASSIS_TRACKWIDTH_METERS = 0.62685;
        public static final double SWERVE_CHASSIS_WHEELBASE_METERS = 0.59825;
        public static final double SWERVE_WHEEL_CIRCUMFERENCE = Conversions.inchesToMeters(4.0) * Math.PI;
        public static final double SWERVE_MAX_SPEED = 4.7; //Wait for test.
        public static final double SWERVE_MAX_ANGULAR_VELOCITY = 4.7;//Wait for test.

        public static final double SWERVE_POV_MOVE_SPEED = 0.2;


        public static final int SWERVE_ANGLE_CURRENT_LIMIT = 40;
        public static final double SWERVE_ANGLE_PEAK_CURRENT_LIMIT = 60;
        public static final double SWERVE_ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean SWERVE_ANGLE_CURRENT_ENABLED = true;

        public static final double SWERVE_DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;
        public static final double SWERVE_DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double SWERVE_DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean SWERVE_DRIVE_CURRENT_ENABLED = true;

        public static final double[] SWERVE_DRIVE_PID = {0.05, 0.0, 0.0}; // TO DO : Using Tuner.
        public static final double SWERVE_DRIVE_KS = (0.0/MAX_VOLTAGE);
        public static final double SWERVE_DRIVE_KV = (2.3092/MAX_VOLTAGE);
        public static final double SWERVE_DRIVE_KA = (1.2331/MAX_VOLTAGE);



        public static final double SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP = 0.0;


        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final SwerveDriveKinematics L2_SwerveDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, SWERVE_CHASSIS_WHEELBASE_METERS / 2),
                new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, SWERVE_CHASSIS_WHEELBASE_METERS / 2),
                new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, -SWERVE_CHASSIS_WHEELBASE_METERS / 2),
                new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, -SWERVE_CHASSIS_WHEELBASE_METERS / 2)
        );

    }

    public static final class AutoConstants{
        //public static final double SWERVE_AUTO_XY_PID[] = {5.0, 0.0, 0.0}; // TODO : Using Tuner.
        //public static final double SWERVE_AUTO_Z_PID[] = {5.0, 0.0, 0.0}; // TODO : Using Tuner.

        public static final HolonomicPathFollowerConfig SwervePathFollower = new HolonomicPathFollowerConfig(
                new PIDConstants(0.05,0.0,0.0), // TODO : Using Tuner for XY.
                new PIDConstants(0.05,0.0,0.0), // TODO : Using Tuner for Rotate.
                SwerveConstants.SWERVE_MAX_SPEED, // MaxSpeed in m/s
                0.4, // DriveBaseRadius
                new ReplanningConfig()
        );
    }
    public static final class CollectConstants{
        public static final double[] COLLECT_PID = {0.0045,0,0.0006,0.088};// TO DO : Using Tuner.
        public static final int COLLECT_CURRENT_LIMIT = 38;
        public static final CANSparkBase.IdleMode COLLECT_NEUTRAL_MODE = CANSparkBase.IdleMode.kCoast;
        public static final double COLLECT_GEAR_RATIO = 1.0/15.0;

        public static final double COLLECT_SPEED = 9.0; // Rotates Per Second
        public static final double REVERSE_AMP_SPEED = -5.0; // Rotates Per Second
        public static final double REVERSE_SHOOT_SPEED = -5.0; // Rotates Per Second

    }
    public static final class ShootConstants{

        public static final double SHOOT_GEAR_RATIO = 2.0;
        public final static double[] SHOOT_PID = {0.003, 0.0, 0.0005,0.225};// TO DO : Using Tuner.
        public static final boolean SHOOT_INVERTED = true;
        public static final double SHOOT_SPEED = 35; // 40 rotations per second.
        public static final double SHOOT_REVERSE_SPEED = -5;
        public static final int SHOOT_CURRENT_LIMIT = 40;
        public static final NeutralModeValue SHOOT_NETURAL_MODE = NeutralModeValue.Coast;

    }



    public static final class ArmConstants{
        //TODO : set PID DOWN for slot 1
        public static final double[] ARM_UP_PID = {0.09307, 0.0, 0.0509011};
        public static final double[] ARM_DOWN_PID = {0.01291,0.0,0.00509011};
        public static final double[] WRIST_UP_PID = {0.051312, 0, 0.001753,0.0};
        public static final double[] WRIST_DOWN_PID = {0.051312, 0, 0.001753,0.0};
        public static final int ARM_UP_SLOT = 0;
        public static final int ARM_DOWN_SLOT = 1;
        public static final int WRIST_UO_SLOT = 0;
        public static final int WRIST_DOWN_SLOT = 1;

        public static final double ARM_KG = 0.64071/MAX_VOLTAGE;
        public static final double ARM_KS = 0.10542/MAX_VOLTAGE;
        public static final double ARM_KV = 0.092035/MAX_VOLTAGE;
        public static final double ARM_KA = 0.0023627/MAX_VOLTAGE;


        public static final double WRIST_KG = 0.12883/MAX_VOLTAGE;
        public static final double WRIST_KS = 0.10925/MAX_VOLTAGE;
        public static final double WRIST_KV = 0.05521/MAX_VOLTAGE;
        public static final double WRIST_KA = 0.0012087/MAX_VOLTAGE;

        public static final double ARM_OFFSET = 10.837714;
        public static final double WRIST_OFFSET = 83.04471;

        public static final int ARM_CURRENT_LIMIT = 40;
        public static final int WRIST_CURRENT_LIMIT = 40;

        public static final double ARM_GEAR_RATIO = 0.004; // 1/250.0
        public static final double WRIST_GEAR_RATIO =1.0/187.5;
        public static final float ARM_FORWARD_LIMIT = 94.7f;
        public static final float ARM_REVERSE_LIMIT = 0f;
        public static final float WRIST_FORWARD_LIMIT = 160f;
        public static final float WRIST_REVERSE_LIMIT = 0f;
        public static final boolean ARM_LEFT_INVERTED = true;
        public static final boolean ARM_RIGHT_INVERTED = false;
        public static final boolean WRIST_INVERTED = true;
        public static final boolean ARM_ENCODER_INVERTED = false;
        public static final boolean WRIST_ENCODER_INVERTED = false;

        public static final double ARM_CONTROL_WAITTIME = 0.1;
        public static final double WRIST_CONTROL_WAITTIME = 0.2;

        public static final float ARM_CONTROL_VALUE = 1.0f;
        public static final float WRIST_CONTROL_VALUE = 0.4f;

        public enum ARM_POSITIONS {
            // TODO : Source,FORWARD_SPEAKER Position TBD to use.
            AMP(50.974227,115.152642),
            SPEAKER(23.179,124.067119),
            COLLECT(7.6614913,111.3801208),
            SOURCE(46.36526,160.35238),
            FORWARD_SPEAKER(0,0),
            NORMAL(41.2,62.63),
            HANG(87.1,120.0);
            private double ArmPosition;
            private double WristPosition;
            private ARM_POSITIONS(double ArmPosition,double WristPosition){
                this.ArmPosition = ArmPosition;
                this.WristPosition = WristPosition;

            }
            public double getArmPosition(){
                return ArmPosition;
            }
            public double getWristPosition(){return WristPosition;}

        }
    }

    public static final class HangConstants{
        public static final double[] HANG_PID = {0.005, 0.0, 0.0005,0.685};// TO DO : Using Tuner.

        public static final int HANG_CURRENT_LIMIT = 35;
        public static final CANSparkBase.IdleMode HANG_NEUTRAL_MODE = CANSparkBase.IdleMode.kBrake;
        public static final double HANG_GEAR_RATIO =1.0/64.0;
        public static final boolean HANG_LEFT_INVERTED = true;
        public static final boolean HANG_RIGHT_INVERTED = false;
        public static final double HANG_SPEED = 1.0; // Rotates Per Second
    }



    public static final class VisionConstants{
        public static final double VISION_POSE_TRUST_WORTHINESS = 1;
        public static final double[] VISION_AIM_PID = {0.01,0,0};

        public static final double VISION_AIM_KP = 0.05;
        public static final double VISION_AIM_KI = 0;
        public static final double VISION_AIM_KD = 0;
        public static final double VISION_AIM_TOLERANCE = 0;
        public static final double VISION_AIM_INTEGRATOR_RANGE = 0.5;
    }





}
