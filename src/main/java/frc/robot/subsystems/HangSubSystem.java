package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HangConstants.*;
import static frc.robot.RobotMap.HangMap.*;

public class HangSubSystem extends SubsystemBase {
    private CANSparkMax mHangLeftMotor = new CANSparkMax(HANG_LEFT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax mHangRightMotor = new CANSparkMax(HANG_RIGHT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);

    public HangSubSystem(){
        configHang(mHangLeftMotor,HANG_LEFT_INVERTED);
        configHang(mHangRightMotor,HANG_RIGHT_INVERTED);
    }

    private void configHang(CANSparkMax motor, boolean inversion){
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
        stopHangMotor(); // To ensure Hang Motor is stopped.
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
