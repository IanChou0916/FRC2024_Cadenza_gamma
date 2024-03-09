package frc.lib.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;

public class MotorConfig {
    private final int canId;
    private final String canBus;
    private final double[] pidf;
    private final double gearRatio;
    private final ConversionUnitType conversionUnitType;
    private final int limitCurrent;
    private final boolean inversion;
    private final Mode mode;

    public enum Mode {
        Coast(true),
        Brake(false);
        private boolean mode;
        Mode(boolean mode) {
            this.mode = mode;
        }
        public NeutralModeValue getTalonMode(){
            return mode ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        }
        public CANSparkBase.IdleMode getSparkMode(){
            return mode ? CANSparkBase.IdleMode.kCoast : CANSparkBase.IdleMode.kBrake;
        }
    }
    public enum ConversionUnitType {
        DEGREES(360.0),
        RADIANS(2 * Math.PI),
        ROTATIONS(1);
        private double unitFactor;
        ConversionUnitType(double unitFactor){
            this.unitFactor = unitFactor;
        }
        public double getUnitFactor(){
            return unitFactor;
        }
    }
    public MotorConfig(int canId, String canBus, double gearRatio, ConversionUnitType conversionUnitType, double[] pidf, int limitCurrent, boolean inversion, Mode mode
    ) {
        this.canId = canId;
        this.canBus = canBus;
        this.gearRatio = gearRatio;
        this.conversionUnitType = conversionUnitType;
        this.pidf = pidf;
        this.limitCurrent = limitCurrent;
        this.inversion = inversion;
        this.mode = mode;

    }
    public MotorConfig(int canId,double gearRatio,ConversionUnitType conversionUnitType,double[] pidf, int limitCurrent, boolean inversion, Mode mode
    ) {
        this(canId, "rio", gearRatio,conversionUnitType, pidf, limitCurrent, inversion, mode);
    }
    public MotorConfig(int canId,double gearRatio,ConversionUnitType conversionUnitType,double[] pidf, int limitCurrent, boolean inversion
    ) {
        this(canId, "rio", gearRatio,conversionUnitType, pidf, limitCurrent, inversion, Mode.Brake);
    }
    public MotorConfig(int canId,double gearRatio,double[] pidf, int limitCurrent, boolean inversion
    ) {
        this(canId, "rio", gearRatio,ConversionUnitType.ROTATIONS, pidf, limitCurrent, inversion, Mode.Brake);
    }
    public MotorConfig(int canId,double gearRatio,double[] pidf, boolean inversion
    ) {
        this(canId, "rio", gearRatio,ConversionUnitType.ROTATIONS, pidf, 37, inversion, Mode.Brake);
    }
    public TalonFX createTalon(){
        TalonFX talon = new TalonFX(canId);
        talon.getConfigurator().apply(getTalonConfig());
        return talon;
    }
    public TalonFXConfiguration getTalonConfig(){
        TalonFXConfiguration talonFXconfig = new TalonFXConfiguration();
        setConversionFactor(talonFXconfig,gearRatio);
        updatePID(talonFXconfig, pidf);
        if( limitCurrent > 0){
            updateSupplyCurrentLimit(talonFXconfig, limitCurrent);
        }
        talonFXconfig.MotorOutput.NeutralMode = mode.getTalonMode();
        talonFXconfig.MotorOutput.Inverted = inversion ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        return talonFXconfig;
    }
    public CANSparkMax createSparkMax(CANSparkLowLevel.MotorType type){
        CANSparkMax motor =  new CANSparkMax(canId, type);
        setCANSparkMaxConfig(motor, type);
        return motor;
    }
    public CANSparkMax createSparkMax(){
        return createSparkMax(CANSparkLowLevel.MotorType.kBrushless);
    }

    public void setCANSparkMaxConfig(CANSparkMax motor, CANSparkLowLevel.MotorType type){
        motor.restoreFactoryDefaults();
        setConversionFactor(motor, conversionUnitType, gearRatio);
        setPID(motor);
        if( limitCurrent > 0){
            createSupplyCurrentLimit(limitCurrent);
        }
        motor.setIdleMode(mode.getSparkMode());
        motor.setInverted(inversion);
        motor.burnFlash();

    }

    public static void setConversionFactor(CANSparkBase motor, ConversionUnitType conversionType, double gearRatio){
        motor.getEncoder().setPositionConversionFactor(conversionType.getUnitFactor()*gearRatio);
        motor.getEncoder().setVelocityConversionFactor(conversionType.getUnitFactor()*gearRatio / 60.0);
    }
    public static TalonFXConfiguration setConversionFactor(TalonFXConfiguration config, double gearRatio){
        config.Feedback.SensorToMechanismRatio = gearRatio;
        return config;
    }
    public static TalonFXConfiguration setDegreeConversionFactor(TalonFXConfiguration config, double gearRatio){
        return setConversionFactor(config, 360.0/gearRatio);
    }

    public static CurrentLimitsConfigs createSupplyCurrentLimit(int limit){
        CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
        currentLimit.StatorCurrentLimit = limit;
        currentLimit.SupplyTimeThreshold = limit;
        currentLimit.SupplyCurrentThreshold = 0;
        currentLimit.StatorCurrentLimitEnable = true;
        return currentLimit;
    }
    public static void updateSupplyCurrentLimit(TalonFXConfiguration config, int currentLimit){
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyTimeThreshold = currentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = currentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    public void setPID(TalonFX talon){
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = pidf[0];
        slot0Configs.kI = pidf[1];
        slot0Configs.kD = pidf[2];
        slot0Configs.kV = pidf[3];
        talon.getConfigurator().apply(slot0Configs);
    }
    public void updatePID(TalonFXConfiguration config, double[] pidf){
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = pidf[0];
        slot0Configs.kI = pidf[1];
        slot0Configs.kD = pidf[2];
        slot0Configs.kV = pidf[3];
        config.withSlot0(slot0Configs);
    }

    public void setPID(CANSparkMax neo){
       neo.getPIDController().setP(pidf[0],0);
       neo.getPIDController().setI(pidf[1],0);
       neo.getPIDController().setD(pidf[2],0);
       neo.getPIDController().setFF(pidf[3],0);
    }
    public PIDController getPIDController(){
        return new PIDController(pidf[0], pidf[1], pidf[2]);
    }
}
