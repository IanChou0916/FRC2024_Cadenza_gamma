package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.convert.Conversions;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.RobotMap.ArmMap.*;

public class ArmSubSystem extends SubsystemBase {
    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;
    private CANSparkFlex mWristMotor;
    private AbsoluteEncoder mWristAbsoulteEncoder;
    private AbsoluteEncoder mArmAbsoluteEncoder;
    private ArmFeedforward armFeedForward = new ArmFeedforward(ARM_KS,ARM_KG,ARM_KV,ARM_KA);
    private ArmFeedforward wristFeedForward = new ArmFeedforward(WRIST_KS,WRIST_KG,WRIST_KV,WRIST_KA);
    private double armValue = 0;
    private double wristValue = 0;

    public ArmSubSystem(){
        mLeftArmMotor = new CANSparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless);
        mWristMotor = new CANSparkFlex(WRIST_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
        mArmAbsoluteEncoder = mRightArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mWristAbsoulteEncoder = mWristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        configWristMotor();
        mArmAbsoluteEncoder.setZeroOffset(71.9845427);
        mArmAbsoluteEncoder.setPositionConversionFactor(360);
        mArmAbsoluteEncoder.setInverted(ARM_ENCODER_INVERTED);
        configArmMotor(mRightArmMotor,ARM_LEFT_INVERTED);
        configArmMotor(mLeftArmMotor,ARM_RIGHT_INVERTED);

        Timer.delay(0.5);
        syncAbsoluteEncoder();
    }

    private void configArmMotor(CANSparkMax armMotor,boolean inversion){
        armMotor.restoreFactoryDefaults();
        armMotor.clearFaults();
        
        armMotor.getEncoder().setPositionConversionFactor(360*ARM_GEAR_RATIO);
        mArmAbsoluteEncoder.setZeroOffset(71.9845427);
        mArmAbsoluteEncoder.setPositionConversionFactor(360);
        mArmAbsoluteEncoder.setInverted(ARM_ENCODER_INVERTED);
        
        armMotor.getPIDController().setP(ARM_PID[0], 0);
        armMotor.getPIDController().setI(ARM_PID[1], 0);
        armMotor.getPIDController().setD(ARM_PID[2], 0);
        //mRightArMotor.getPIDController().setFF(, 0);
        
        armMotor.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
        //mRightArmMotor.setSoftLimit(SoftLimitDirection.kForward, ARM_FORWARD_LIMIT);
        //mRightArmMotor.setSoftLimit(SoftLimitDirection.kReverse, ARM_REVERSE_LIMIT);

        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setInverted(inversion);

        armMotor.burnFlash();

        armMotor.getEncoder().setPosition(0);

    }

    private void configWristMotor(){
        mWristMotor.restoreFactoryDefaults();
        mWristMotor.clearFaults();
        
        mWristMotor.getEncoder().setPositionConversionFactor(360*WRIST_GEAR_RATIO);
        mWristAbsoulteEncoder.setPositionConversionFactor(360);
        mWristAbsoulteEncoder.setZeroOffset(331.8981314);
        
        mWristMotor.getPIDController().setP(WRIST_PID[0], 0);
        mWristMotor.getPIDController().setI(WRIST_PID[1], 0);
        mWristMotor.getPIDController().setD(WRIST_PID[2], 0);
        //mWristMotor.getPIDController().setFF(WRIST_PID[3], 0);
        
        mWristMotor.setSmartCurrentLimit(WRIST_CURRENT_LIMIT);
        mWristMotor.setSoftLimit(SoftLimitDirection.kForward, WRIST_FORWARD_LIMIT);
        mWristMotor.setSoftLimit(SoftLimitDirection.kReverse, WRIST_REVERSE_LIMIT);
        mWristMotor.enableSoftLimit(SoftLimitDirection.kForward,true);
        mWristMotor.enableSoftLimit(SoftLimitDirection.kReverse,true);

        mWristMotor.setIdleMode(IdleMode.kBrake);
        //mWristMotor.getEncoder().setInverted(true);
        mWristAbsoulteEncoder.setInverted(WRIST_ENCODER_INVERTED);

        mWristMotor.setInverted(WRIST_INVERTED); // wrist motor
        mWristMotor.burnFlash();

        mWristMotor.getEncoder().setPosition(0);
    }

    private void syncAbsoluteEncoder(){
        mLeftArmMotor.getEncoder().setPosition(mArmAbsoluteEncoder.getPosition());
        mRightArmMotor.getEncoder().setPosition(mArmAbsoluteEncoder.getPosition());
        mWristMotor.getEncoder().setPosition(mWristAbsoulteEncoder.getPosition());
        armValue = mArmAbsoluteEncoder.getPosition();
        wristValue = mWristAbsoulteEncoder.getPosition();

    }
    public void setArmPosition(double degrees) {
        mLeftArmMotor.getPIDController().setReference(degrees, CANSparkBase.ControlType.kPosition, 0, armFeedForward.calculate(degrees, 0));
        mRightArmMotor.getPIDController().setReference(degrees, CANSparkBase.ControlType.kPosition,0,armFeedForward.calculate(degrees,0));
    }
    public void setWristPosition(double degrees){
        double groundDegrees = degrees+mArmAbsoluteEncoder.getPosition()-90;
        mWristMotor.getPIDController().setReference(degrees,CANSparkFlex.ControlType.kPosition,0,wristFeedForward.calculate(groundDegrees,0));
    }


    public double getArmAngle(){
        return mArmAbsoluteEncoder.getPosition();
    }
    public double getWristAngle(){
        return mWristAbsoulteEncoder.getPosition();
    }




    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Arm Position", mLeftArmMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Arm Position", mRightArmMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Position", mWristMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Encoder",mArmAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Encoder",mWristAbsoulteEncoder.getPosition());
    }
    
}
