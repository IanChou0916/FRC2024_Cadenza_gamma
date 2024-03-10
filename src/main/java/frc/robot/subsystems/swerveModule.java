package frc.robot.subsystems;



import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.convert.Conversions;
import frc.lib.util.ModuleState;
import frc.lib.util.SwerveTypeConstants;
import frc.robot.Constants;
import frc.robot.RobotMap;
import static frc.robot.Constants.SwerveConstants.*;


public class swerveModule {

    public int moduleNumber;
    private SwerveTypeConstants swerveTypeConstants;
    private Rotation2d angleOffset ;


    private TalonFX mDriveFalcon;
    private TalonFX mAngleFalcon;
    private CANcoder mAngleCanCoder;


    //private RelativeEncoder mAngleEncoder;
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocityVoltage = new VelocityVoltage(0);
    private final PositionVoltage anglePositionDutyCycle = new PositionVoltage(0);

    //private final PositionVoltage ANGLE_POSITION = new PositionVoltage(0);

    private final SimpleMotorFeedforward fMotorFeedforward = new SimpleMotorFeedforward(
            SWERVE_DRIVE_KS, SWERVE_DRIVE_KV, SWERVE_DRIVE_KA);

    public swerveModule(int moduleNumber,
                        SwerveTypeConstants swerveTypeConstants,
                        int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset){
        this.moduleNumber = moduleNumber;
        this.swerveTypeConstants = swerveTypeConstants;
        this.angleOffset = angleOffset;

        mAngleCanCoder = new CANcoder(canCoderID,RobotMap.SWERVE_CANBUS_TYPE);
        mAngleCanCoderConfig();

        mDriveFalcon = new TalonFX(driveMotorID, RobotMap.SWERVE_CANBUS_TYPE);
        mDriveConfig();
        mAngleFalcon = new TalonFX(angleMotorID, RobotMap.SWERVE_CANBUS_TYPE);
        mAngleConfig();
        resetToAbosolute();
        //mAngleEncoder = mAngleNeo.getEncoder();

        //lastAngle = getState().angle;

    }

    public void setDesireState(SwerveModuleState desiredState){
        desiredState = ModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState);
    }



    private void setSpeed(SwerveModuleState desiredState){
        double velocity = Conversions.MPSToFalcon(
                desiredState.speedMetersPerSecond,
                swerveTypeConstants.wheelCircumference,
                1/swerveTypeConstants.driveGearRadio);
        //driveVelocityVoltage.Velocity = desiredState.speedMetersPerSecond;
        //driveVelocityVoltage.FeedForward = fMotorFeedforward.calculate(desiredState.speedMetersPerSecond);
        //mDriveFalcon.setControl(driveVelocityVoltage);
        mDriveFalcon.setControl(driveVelocityVoltage.withVelocity(velocity));
    }

    private void setAngle(SwerveModuleState desiredState){
//        System.out.println(desiredState.angle.getDegrees());
        mAngleFalcon.setControl(anglePositionDutyCycle.withPosition(desiredState.angle.getRotations()));
//        mAngleFalcon.setControl(anglePositionDutyCycle.withPosition(1));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(mDriveFalcon.getPosition().getValue(), swerveTypeConstants.wheelCircumference, swerveTypeConstants.driveGearRadio),
                getAngle()
        );
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                Conversions.falconToMeters(mDriveFalcon.getPosition().getValue(), swerveTypeConstants.wheelCircumference, swerveTypeConstants.driveGearRadio),
                getAngle()
        );
    }


    private Rotation2d getAngle(){
        //System.out.printf("%.2f",mRelativeEncoder.getPosition());
//        if (mAngleFalcon.getDeviceID() == 2)System.out.println(mAngleFalcon.getPosition().getValue());
        return Rotation2d.fromRotations(mAngleFalcon.getPosition().getValue()%1);
        // Rotation.fromDegrees(Convertions.falconToDegrees(mAngleFalcon.getPosition().getValue(),1));
    }

    public Rotation2d getCanCoder(){
//        if(mAngleCanCoder.getDeviceID() == 1)System.out.println(mAngleCanCoder.getAbsolutePosition().getValue());
        return Rotation2d.fromRotations(mAngleCanCoder.getAbsolutePosition().getValue());
    }


    private void mDriveConfig(){
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Slot0.kP = SWERVE_DRIVE_PID[0];
        driveConfig.Slot0.kI = SWERVE_DRIVE_PID[1];
        driveConfig.Slot0.kD = SWERVE_DRIVE_PID[2];
        driveConfig.Slot0.kS = SWERVE_DRIVE_KS;
        driveConfig.Slot0.kV = SWERVE_DRIVE_KV;
        driveConfig.Slot0.kA = SWERVE_DRIVE_KA;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = SWERVE_DRIVE_CURRENT_ENABLED;
        driveConfig.CurrentLimits.StatorCurrentLimit = SWERVE_DRIVE_CONTINUOUS_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = SWERVE_DRIVE_PEAK_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyTimeThreshold = SWERVE_DRIVE_PEAK_CURRENT_DURATION;



        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP;

        driveConfig.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;
        driveConfig.MotorOutput.Inverted = swerveTypeConstants.driveMotorInverted;

        driveConfig.Feedback.SensorToMechanismRatio = swerveTypeConstants.driveGearRadio;
        mDriveFalcon.getConfigurator().apply(driveConfig);
    }
    public void resetToAbosolute(){
        double absolute = (getCanCoder().getDegrees() - angleOffset.getDegrees());
        //System.out.printf("%.2f",absolute);
        mAngleFalcon.setPosition(Conversions.degreesToFalcon(absolute));
    }
    public void stop() {
        mDriveFalcon.stopMotor();
        mAngleFalcon.stopMotor();
    }

    private void mAngleConfig(){
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.Slot0.kP = swerveTypeConstants.anglePIDF[0];
        angleConfig.Slot0.kI = swerveTypeConstants.anglePIDF[1];
        angleConfig.Slot0.kD = swerveTypeConstants.anglePIDF[2];

        angleConfig.CurrentLimits.StatorCurrentLimitEnable = SWERVE_ANGLE_CURRENT_ENABLED;
        angleConfig.CurrentLimits.StatorCurrentLimit = SWERVE_ANGLE_CURRENT_LIMIT;
        angleConfig.CurrentLimits.SupplyCurrentThreshold = SWERVE_ANGLE_PEAK_CURRENT_LIMIT;
        angleConfig.CurrentLimits.SupplyTimeThreshold = SWERVE_ANGLE_PEAK_CURRENT_DURATION;

        angleConfig.MotorOutput.NeutralMode = ANGLE_NEUTRAL_MODE;
        angleConfig.MotorOutput.Inverted = swerveTypeConstants.angleMotorInverted;

        angleConfig.Feedback.SensorToMechanismRatio = swerveTypeConstants.angleGearRadio;
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        mAngleFalcon.getConfigurator().apply(angleConfig);



    }

    private void mAngleCanCoderConfig(){
        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor.SensorDirection =SensorDirectionValue.CounterClockwise_Positive;
        canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        //canConfig.MagnetSensor.MagnetOffset = 0.0;
        mAngleCanCoder.getConfigurator().apply(canConfig);

    }



}