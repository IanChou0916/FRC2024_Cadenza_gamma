package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.AMP;
import static frc.robot.RobotMap.ArmMap.*;

public class ArmSubSystem extends SubsystemBase {
    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;
    private CANSparkFlex mWristMotor;
    private AbsoluteEncoder mWristAbsoulteEncoder;
    private AbsoluteEncoder mArmAbsoluteEncoder;
    private ArmFeedforward armFeedForward = new ArmFeedforward(ARM_KS,ARM_KG,ARM_KV,ARM_KA);
    private ArmFeedforward wristFeedForward = new ArmFeedforward(WRIST_KS,WRIST_KG,WRIST_KV,WRIST_KA);

    public ArmSubSystem(){
        mLeftArmMotor = new CANSparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless);
        mWristMotor = new CANSparkFlex(WRIST_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
        mArmAbsoluteEncoder = mRightArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mWristAbsoulteEncoder = mWristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        configWristMotor();
        mArmAbsoluteEncoder.setZeroOffset(ARM_OFFSET);
        mArmAbsoluteEncoder.setPositionConversionFactor(360);
        mArmAbsoluteEncoder.setInverted(ARM_ENCODER_INVERTED);
        configArmMotor(mRightArmMotor,ARM_LEFT_INVERTED);
        configArmMotor(mLeftArmMotor,ARM_RIGHT_INVERTED);
        Timer.delay(0.5);
        syncAbsoluteEncoder();
        //resetToAMP();
    }

    private void configArmMotor(CANSparkMax armMotor,boolean inversion){
        armMotor.restoreFactoryDefaults();
        armMotor.clearFaults();
        
        armMotor.getEncoder().setPositionConversionFactor(360*ARM_GEAR_RATIO);

        mArmAbsoluteEncoder.setZeroOffset(ARM_OFFSET);
        mArmAbsoluteEncoder.setPositionConversionFactor(360);
        mArmAbsoluteEncoder.setInverted(ARM_ENCODER_INVERTED);
        
        armMotor.getPIDController().setP(ARM_UP_PID[0], ARM_UP_SLOT);
        armMotor.getPIDController().setI(ARM_UP_PID[1], ARM_UP_SLOT);
        armMotor.getPIDController().setD(ARM_UP_PID[2], ARM_UP_SLOT);

        armMotor.getPIDController().setP(ARM_DOWN_PID[0],ARM_DOWN_SLOT);
        armMotor.getPIDController().setI(ARM_DOWN_PID[1],ARM_DOWN_SLOT);
        armMotor.getPIDController().setD(ARM_DOWN_PID[2],ARM_DOWN_SLOT);


        armMotor.setSmartCurrentLimit(ARM_CURRENT_LIMIT);

        armMotor.setInverted(inversion);

        armMotor.setSoftLimit(SoftLimitDirection.kForward, ARM_FORWARD_LIMIT);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, ARM_REVERSE_LIMIT);
        armMotor.enableSoftLimit(SoftLimitDirection.kForward,true);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse,true);

        armMotor.setIdleMode(IdleMode.kBrake);


        armMotor.burnFlash();

        armMotor.getEncoder().setPosition(0);

    }

    private void configWristMotor(){
        mWristMotor.restoreFactoryDefaults();
        mWristMotor.clearFaults();
        
        mWristMotor.getEncoder().setPositionConversionFactor(360*WRIST_GEAR_RATIO);
        mWristAbsoulteEncoder.setPositionConversionFactor(360);
        mWristAbsoulteEncoder.setZeroOffset(WRIST_OFFSET);
        
        mWristMotor.getPIDController().setP(WRIST_PID[0], 0);
        mWristMotor.getPIDController().setI(WRIST_PID[1], 0);
        mWristMotor.getPIDController().setD(WRIST_PID[2], 0);

        
        mWristMotor.setSmartCurrentLimit(WRIST_CURRENT_LIMIT);
        mWristMotor.setSoftLimit(SoftLimitDirection.kForward, WRIST_FORWARD_LIMIT);
        mWristMotor.setSoftLimit(SoftLimitDirection.kReverse, WRIST_REVERSE_LIMIT);
        mWristMotor.enableSoftLimit(SoftLimitDirection.kForward,true);
        mWristMotor.enableSoftLimit(SoftLimitDirection.kReverse,true);

        mWristMotor.setIdleMode(IdleMode.kBrake);

        mWristAbsoulteEncoder.setInverted(WRIST_ENCODER_INVERTED);

        mWristMotor.setInverted(WRIST_INVERTED); // wrist motor
        mWristMotor.burnFlash();

        mWristMotor.getEncoder().setPosition(0);
    }

    private void syncAbsoluteEncoder(){
        mLeftArmMotor.getEncoder().setPosition(mArmAbsoluteEncoder.getPosition());
        mRightArmMotor.getEncoder().setPosition(mArmAbsoluteEncoder.getPosition());
        mWristMotor.getEncoder().setPosition(mWristAbsoulteEncoder.getPosition());

    }
    public void setArmPosition(double degrees) {
        if(getArmAngle() > degrees){ // To the floor, Arm down
            mLeftArmMotor.getPIDController().setReference(degrees, CANSparkMax.ControlType.kPosition,
                    ARM_DOWN_SLOT, armFeedForward.calculate(degrees, 0));
            mRightArmMotor.getPIDController().setReference(degrees, CANSparkMax.ControlType.kPosition,
                    ARM_DOWN_SLOT, armFeedForward.calculate(degrees,0));
        }
        else { // To the top, Arm up.
            mLeftArmMotor.getPIDController().setReference(degrees, CANSparkMax.ControlType.kPosition,
                    ARM_UP_SLOT, armFeedForward.calculate(degrees, 0));
            mRightArmMotor.getPIDController().setReference(degrees, CANSparkMax.ControlType.kPosition,
                    ARM_UP_SLOT, armFeedForward.calculate(degrees, 0));
        }

    }
    public void setArmPosition(ARM_POSITIONS position) {
        if(getArmAngle() > position.getArmPosition()){ // To the floor, Arm down
            mLeftArmMotor.getPIDController().setReference(position.getArmPosition(),
                    CANSparkMax.ControlType.kPosition, ARM_DOWN_SLOT,
                    armFeedForward.calculate(position.getArmPosition(), 0));
            mRightArmMotor.getPIDController().setReference(position.getArmPosition(),
                    CANSparkMax.ControlType.kPosition,ARM_DOWN_SLOT,
                    armFeedForward.calculate(position.getArmPosition(), 0));
        }
        else { // To the top, Arm up.
            mLeftArmMotor.getPIDController().setReference(position.getArmPosition(),
                    CANSparkMax.ControlType.kPosition, ARM_UP_SLOT,
                    armFeedForward.calculate(position.getArmPosition(), 0));
            mRightArmMotor.getPIDController().setReference(position.getArmPosition(),
                    CANSparkMax.ControlType.kPosition,ARM_UP_SLOT,
                    armFeedForward.calculate(position.getArmPosition(), 0));
        }
    }
    public void setWristPosition(double degrees){
        double groundDegrees = degrees+mArmAbsoluteEncoder.getPosition()-90;
        mWristMotor.getPIDController().setReference(degrees,CANSparkFlex.ControlType.kPosition,0,wristFeedForward.calculate(groundDegrees,0));
    }
    public void setWristPosition(ARM_POSITIONS position){
        double groundDegrees = position.getWristPosition()+mArmAbsoluteEncoder.getPosition()-90;
        mWristMotor.getPIDController().setReference(position.getWristPosition(), CANSparkFlex.ControlType.kPosition,0,wristFeedForward.calculate(groundDegrees,0));
    }


    public double getArmAngle(){
        return mArmAbsoluteEncoder.getPosition();
    }
    public double getWristAngle(){
        return mWristAbsoulteEncoder.getPosition();
    }

    private void resetToAMP(){
        if(getArmAngle() > 50){
            setWristPosition(AMP);
            if(Math.abs(AMP.getWristPosition() - getWristAngle()) < 15){
                setArmPosition(AMP);
            }
        }
        else {
            setArmPosition(AMP);
            setWristPosition(AMP);
        }

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
