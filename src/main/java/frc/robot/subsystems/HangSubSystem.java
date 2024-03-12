package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HangConstants.*;
import static frc.robot.RobotMap.HangMap.*;

public class HangSubSystem extends SubsystemBase {
    private final CANSparkFlex mHangLeftMotor = new CANSparkFlex(HANG_LEFT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex mHangRightMotor = new CANSparkFlex(HANG_RIGHT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    private SparkLimitSwitch mLeftHangSwitch;
    private SparkLimitSwitch mRightHangSwitch;

    public HangSubSystem(){
        configHang(mHangLeftMotor,HANG_LEFT_INVERTED);
        configHang(mHangRightMotor,HANG_RIGHT_INVERTED);

        stopHangMotor(); // To ensure Hang Motor is stopped.
    }

    private void configHang(CANSparkFlex motor, boolean inversion){
        motor.restoreFactoryDefaults();

        motor.getEncoder().setPositionConversionFactor(HANG_GEAR_RATIO);
        motor.getEncoder().setVelocityConversionFactor(HANG_GEAR_RATIO / 60.0); // RPS

        motor.getPIDController().setP(HANG_PID[0], 0);
        motor.getPIDController().setI(HANG_PID[1], 0);
        motor.getPIDController().setD(HANG_PID[2], 0);
        motor.getPIDController().setFF(HANG_PID[3], 0);

        motor.setSmartCurrentLimit(HANG_CURRENT_LIMIT);
        motor.setIdleMode(HANG_NEUTRAL_MODE);


        motor.setInverted(inversion);
        motor.burnFlash();
        motor.getEncoder().setPosition(0);

    }
    public void setLeftHangMotor(double velocity){
        mHangLeftMotor.getPIDController().setReference(velocity, CANSparkBase.ControlType.kVelocity);
    }
    public void setRightHangMotor(double velocity){
        mHangRightMotor.getPIDController().setReference(velocity, CANSparkBase.ControlType.kVelocity);
    }

    public void stopHangMotor(){
        mHangLeftMotor.set(0);
        mHangRightMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("mLeftHang",mHangLeftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("mRightHang",mHangRightMotor.getEncoder().getVelocity());

    }
}
