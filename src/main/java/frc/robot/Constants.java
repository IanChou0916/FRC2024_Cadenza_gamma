package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.convert.Conversions;
import frc.lib.motor.MotorConfig;
import frc.lib.util.SwerveTypeConstants;

import static frc.robot.Constants.ShootConstants.SHOOT_CURRENT_LIMIT;
import static frc.robot.Constants.ShootConstants.SHOOT_PID;

public class Constants {

    public final static class JoyStickConstants {
        public static final double DRIVE_JOYSTICK_DEADBAND = 0.1;
        public static final double OPERATOR_JOYSTICK_DEADBAND = 0.05;
    }
    public static final class SwerveConstants{

    }

    public static boolean NAVX_INVERTED = true;
    public static byte NAVX_UPDATE_RATE = 127;

    public final static double MAX_VOLTAGE = 12.0; 

    public final static Rotation2d FRONT_LEFT_ANGLE_OFFSET = Rotation2d.fromDegrees(328.44726625-180);
    public final static Rotation2d BACK_LEFT_ANGLE_OFFSET = Rotation2d.fromDegrees(271.7578125);
    public final static Rotation2d BACK_RIGHT_ANGLE_OFFSET = Rotation2d.fromDegrees(249.2578125);
    public final static Rotation2d FRONT_RIGHT_ANGLE_OFFSET = Rotation2d.fromDegrees(208.125-180);

    //
    public static final double SWERVE_CHASSIS_TRACKWIDTH_METERS = 0.59825;
    public static final double SWERVE_CHASSIS_WHEELBASE_METERS = 0.62865;
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

    public static final double SWERVE_DRIVE_PID[] = {0.05, 0.0, 0.0}; // TO DO : Using Tuner.
    public static final double SWERVE_DRIVE_KS = (0.32/MAX_VOLTAGE);
    public static final double SWERVE_DRIVE_KV = (1.51/MAX_VOLTAGE);
    public static final double SWERVE_DRIVE_KA = (0.27/MAX_VOLTAGE);



    public static final double SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP = 0.0;


    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final class AutoConstants{
        //public static final double SWERVE_AUTO_XY_PID[] = {5.0, 0.0, 0.0}; // TODO : Using Tuner.
        //public static final double SWERVE_AUTO_Z_PID[] = {5.0, 0.0, 0.0}; // TODO : Using Tuner.

        public static final HolonomicPathFollowerConfig SwervePathFollower = new HolonomicPathFollowerConfig(
                new PIDConstants(0.05,0.0,0.0), // TODO : Using Tuner for XY.
                new PIDConstants(0.05,0.0,0.0), // TODO : Using Tuner for Rotate.
                SWERVE_MAX_SPEED, // MaxSpeed in m/s
                0.4, // DriveBaseRadius
                new ReplanningConfig()
        );
    }



    public static final SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, SWERVE_CHASSIS_WHEELBASE_METERS / 2),
        new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, SWERVE_CHASSIS_WHEELBASE_METERS / 2),
        new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, -SWERVE_CHASSIS_WHEELBASE_METERS / 2),
        new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, -SWERVE_CHASSIS_WHEELBASE_METERS / 2)
    );
    public static final Translation2d flModuleOffset = new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, SWERVE_CHASSIS_WHEELBASE_METERS / 2);

    public static final class CollectConstants{
        public static final double[] COLLECT_PID = {0.0045,0,0.0006,0.088};// TO DO : Using Tuner.
        public static final int COLLECT_CURRENT_LIMIT = 38;
        public static final CANSparkBase.IdleMode COLLECT_NEUTRAL_MODE = CANSparkBase.IdleMode.kBrake;
        public static final double COLLECT_GEAR_RATIO = 1.0/15.0;

        public static final double COLLECT_SPEED = 9.0; // Rotates Per Second
        public static final double REVERSE_AMP_SPEED = -5.0; // Rotates Per Second
        public static final double REVERSE_SHOOT_SPEED = -2.0; // Rotates Per Second
        
    }
    public static final class ShootConstants{



        public static final double SHOOT_GEAR_RATIO = 2.0;
        public final static double[] SHOOT_PID = {0.003, 0.0, 0.0005,0.225};// TO DO : Using Tuner.
        public static final boolean SHOOT_INVERTED = true;
        public static final double SHOOT_SPEED = 1000;
        public static final int SHOOT_CURRENT_LIMIT = 35;
        public static final NeutralModeValue SHOOT_NETURAL_MODE = NeutralModeValue.Coast;

    }



    public static final class ArmConstants{
        public static final double[] ARM_PID = {0.001, 0.0, 0.0,0.0};
        public static final double[] WRIST_PID = {0.001, 0.0, 0.0,0.0};// TO DO : Using Tuner.
        public static final double ARM_KG = 0.28;
        public static final double ARM_KV = 0.09;
        public static final double ARM_KA = 0;
        public static final double ARM_KS = 0.0;

        public static final double WRIST_KG = 0.31;
        public static final double WRIST_KV = 0.06;
        public static final double WRIST_KA = 0;
        public static final double WRIST_KS = 0.0;

        public static final double ARM_GEAR_RATIO = 0.004; // 1/250.0
        public static final double WRIST_GEAR_RATIO =1.0/187.5;
        public static final float ARM_FORWARD_LIMIT = 94.7f;
        public static final float ARM_REVERSE_LIMIT = 0f;
        public static final float WRIST_FORWARD_LIMIT = 160f;
        public static final float WRIST_REVERSE_LIMIT = 0f;
        public static final boolean ARM_LEFT_INVERTED = true ;
        public static final boolean ARM_RIGHT_INVERTED = false;
        public static final boolean WRIST_INVERTED = true;
        public static final boolean ARM_ENCODER_INVERTED = true;
        public static final boolean WRIST_ENCODER_INVERTED = false;

        public static final double ARM_CONTROL_WAITTIME = 0.1;
        public static final double WRIST_CONTROL_WAITTIME = 0.1;

        public static final float ARM_CONTROL_VALUE = 1.0f;
        public static final float WRIST_CONTROL_VALUE = 0.5f;

        public enum ARM_POSITIONS {
            // TODO : Source,FORWARD_SPEAKER Position TBD to use.
            AMP(30.55,114.177),
            SPEAKER(42,28.598),
            COLLECT(90.1164,128.982188061),
            SOURCE(4,0),
            FORWARD_SPEAKER(0,0);
            private double firstArmPosition;
            private double secondArmPosition;
            private ARM_POSITIONS(double firstArmPosition, double secondArmPosition){
                this.firstArmPosition = firstArmPosition;
                this.secondArmPosition = secondArmPosition;

            }
            public double getFirstArmPosition(){
                return firstArmPosition;
            }
            public double getSecondArmPosition(){return secondArmPosition;}

        }
    }

    public static final class HangConstants{
        public static final double[] HANG_PID = {0.005, 0.0, 0.0005,0.685   };// TO DO : Using Tuner.
        public static final double HANG_OPENLOOPRAMP = 0.2;
        public static final double HANG_CLOSELOOPRAMP = 0.0;
        public static final int HANG_CURRENT_LIMIT = 35;
        public static final CANSparkBase.IdleMode HANG_NEUTRAL_MODE = CANSparkBase.IdleMode.kBrake;
        public static final double HANG_GEAR_RATIO =1.0/64.0;
        public static final boolean HANG_LEFT_INVERTED = false;
        public static final boolean HANG_RIGHT_INVERTED = true;
        public static final double HANG_SPEED = 2; // Rotates Per Second
    }



    public static final class VisionConstants{
        public static final double VISION_POSE_TRUST_WORTHINESS = 1;

        public static final double VISION_AIM_KP = 0.01;
        public static final double VISION_AIM_KI = 0;
        public static final double VISION_AIM_KD = 0;
        public static  final  double VISION_AIM_TOLERANCE = 0;
        public static final double VISION_AIM_INTEGRATOR_RANGE = 0.5;
    }





}
