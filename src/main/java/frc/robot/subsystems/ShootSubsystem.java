package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShootConstants.*;
import static frc.robot.RobotMap.ShootMap.LEFT_SHOOTER;
import static frc.robot.RobotMap.ShootMap.RIGHT_SHOOTER;

public class ShootSubsystem extends SubsystemBase {
    private TalonFX mLeftShoooter;
    private TalonFX mRightShooter;
    private final VelocityVoltage shootVoltage = new VelocityVoltage(0);
    private boolean shootEnabled = false;

    public ShootSubsystem(){
        mLeftShoooter = new TalonFX(LEFT_SHOOTER,"rio");
        mRightShooter = new TalonFX(RIGHT_SHOOTER,"rio");
        configShooter(mLeftShoooter,SHOOT_INVERTED);
        configShooter(mRightShooter,SHOOT_INVERTED );
    }
    private void configShooter(TalonFX motor,boolean inversion){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SHOOT_PID[0];
        config.Slot0.kI = SHOOT_PID[1];
        config.Slot0.kD = SHOOT_PID[2];
        config.Slot0.kV = SHOOT_PID[3];

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = SHOOT_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentThreshold = SHOOT_CURRENT_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
        config.MotorOutput.NeutralMode = SHOOT_NETURAL_MODE;
        config.MotorOutput.Inverted = inversion ? InvertedValue.Clockwise_Positive
                :InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = SHOOT_GEAR_RATIO;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        motor.getConfigurator().apply(config);
    }
    public void setSpeed(double velocity){
        mLeftShoooter.setControl(shootVoltage.withVelocity(velocity));
        mRightShooter.setControl(shootVoltage.withVelocity(velocity));
    }
    public void shootNote(){
        mLeftShoooter.setControl(shootVoltage.withVelocity(SHOOT_SPEED));
        mRightShooter.setControl(shootVoltage.withVelocity(SHOOT_SPEED));
        shootEnabled = true;
    }
    public void reverseNote(){
        mLeftShoooter.setControl(shootVoltage.withVelocity(SHOOT_REVERSE_SPEED));
        mRightShooter.setControl(shootVoltage.withVelocity(SHOOT_REVERSE_SPEED));
        shootEnabled = true;
    }
    public void stopShoot(){
        mLeftShoooter.set(0);
        mRightShooter.set(0);
        shootEnabled = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("LeftShooter",mLeftShoooter.getVelocity().getValue());
        SmartDashboard.putNumber("RightShooter",mRightShooter.getVelocity().getValue());
    }




}
