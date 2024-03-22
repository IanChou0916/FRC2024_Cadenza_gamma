
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.NORMAL;
import static frc.robot.Constants.CollectConstants.*;
import static frc.robot.RobotMap.CollectMap.*;
import static frc.robot.Constants.ArmConstants.*;

public class CollectSubsystem extends SubsystemBase {
  private static CANSparkMax mCollectIntake;

  private static RelativeEncoder mCollectEncoder;
  private boolean intakeEnabled = false;
  public boolean isReverse = false;
  private final DigitalInput collectReceiver = new DigitalInput(LASER_DIGITAL_INPUT);
  private final DigitalOutput collectTransmitter = new DigitalOutput(LASER_DIGITAL_OUTPUT);

  private ARM_POSITIONS tuningPosition = NORMAL;

  // TODO : wait until delete Transmitter;



  public CollectSubsystem() {
    mCollectIntake = new CANSparkMax(COLLECT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    mCollectEncoder = mCollectIntake.getEncoder();


    collectConfig();

    collectTransmitter.set(true);
  }

  private void collectConfig() {
    mCollectIntake.restoreFactoryDefaults();
    mCollectIntake.clearFaults();

    mCollectEncoder.setPositionConversionFactor(COLLECT_GEAR_RATIO);
    mCollectEncoder.setVelocityConversionFactor(COLLECT_GEAR_RATIO / 60.0);

    mCollectIntake.getPIDController().setP(COLLECT_PID[0], 0);
    mCollectIntake.getPIDController().setI(COLLECT_PID[1], 0);
    mCollectIntake.getPIDController().setD(COLLECT_PID[2], 0);
    mCollectIntake.getPIDController().setFF(COLLECT_PID[3], 0);
    mCollectIntake.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,false);
    mCollectIntake.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,false);

    mCollectIntake.setSmartCurrentLimit(COLLECT_CURRENT_LIMIT);
    mCollectIntake.setIdleMode(COLLECT_NEUTRAL_MODE);


    mCollectIntake.setInverted(true);
    resetEncoder();
    mCollectIntake.burnFlash();

  }

  public void collectNote() {
    mCollectIntake.getEncoder().setPosition(0);
    mCollectIntake.getPIDController().setReference(COLLECT_SPEED, ControlType.kVelocity);
    intakeEnabled = true;
  }

  public void stopCollect() {
    mCollectIntake.stopMotor();
    intakeEnabled = false;
  }
  public void resetEncoder(){
    mCollectIntake.getEncoder().setPosition(0);
  }

  public void reverseCollect() {
    mCollectIntake.getPIDController().setReference(REVERSE_SHOOT_SPEED, ControlType.kVelocity);

  }
  public void shootAMP(){
    mCollectIntake.getPIDController().setReference(REVERSE_AMP_SPEED, ControlType.kVelocity);

  }
  public boolean getCollectLimit(){
    return collectReceiver.get();
    /**
     * true for getSignal (Light)
     * false for getNote
     */
  }

  public void setTuningPosition(ARM_POSITIONS configPosition){
    configPosition = tuningPosition;
  }
  public ARM_POSITIONS getTuningPosition(){
    return tuningPosition;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Collect Speed", mCollectIntake.getEncoder().getVelocity());
    //SmartDashboard.putNumber("Collect Position", mCollectIntake.getEncoder().getPosition());
    SmartDashboard.putBoolean("Receive",getCollectLimit());
    SmartDashboard.putString("TuningPosition",tuningPosition.toString());
  }

}
