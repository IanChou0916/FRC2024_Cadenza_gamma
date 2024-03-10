package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HangConstants.*;

public class HangSubSystem extends SubsystemBase {
    private CANSparkMax mHangLeftMotor;
    private CANSparkMax mHangRightMotor;

    public HangSubSystem(){
        mHangLeftMotor = new CANSparkMax(5, CANSparkLowLevel.MotorType.kBrushless);
        mHangRightMotor = new CANSparkMax(6, CANSparkLowLevel.MotorType.kBrushless);
        configHang(mHangLeftMotor,true);
        configHang(mHangRightMotor,false);
    }

    private void configHang(CANSparkMax motor, boolean inversion){
        motor.restoreFactoryDefaults();

        motor.getEncoder().setPositionConversionFactor(HANG_GEAR_RATIO);
        motor.getEncoder().setVelocityConversionFactor(HANG_GEAR_RATIO / 60.0);

        motor.getPIDController().setP(HANG_PID[0], 0);
        motor.getPIDController().setI(HANG_PID[1], 0);
        motor.getPIDController().setD(HANG_PID[2], 0);
        motor.getPIDController().setFF(HANG_PID[3], 0);

        motor.setSmartCurrentLimit(HANG_CURRENT_LIMIT);
        motor.setIdleMode(HANG_NEUTRAL_MODE);


        motor.setInverted(inversion);
        motor.burnFlash();
    }

    public void setLeftHangMotor() {
        mHangLeftMotor.getPIDController().setReference(0.5, CANSparkBase.ControlType.kVelocity);
    }
    public void stopLeftHangMotor(){
        mHangLeftMotor.getPIDController().setReference(0, CANSparkBase.ControlType.kVelocity);

    }
    public void reverseLeftHangMotor(){
        mHangLeftMotor.getPIDController().setReference(-0.5, CANSparkBase.ControlType.kVelocity);

    }
    public void setRightHangMotor(){
        mHangRightMotor.getPIDController().setReference(0.5, CANSparkBase.ControlType.kVelocity);
    }
    public void stopRightHangMotor(){

        mHangRightMotor.getPIDController().setReference(0, CANSparkBase.ControlType.kVelocity);
    }
    public void reverseRightHangMotor(){
        mHangRightMotor.getPIDController().setReference(-0.5, CANSparkBase.ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("mLeftHang",mHangLeftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("mRightHang",mHangRightMotor.getEncoder().getVelocity());

    }
}
