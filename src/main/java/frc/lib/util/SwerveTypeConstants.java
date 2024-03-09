package frc.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.lib.convert.Conversions;

public class SwerveTypeConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRadio;
    public final double driveGearRadio;
    public final double anglePIDF[]; // [P, I, D]

    public final InvertedValue angleMotorInverted;
    public final InvertedValue driveMotorInverted;
    public final InvertedValue canCoderInverted;

    public SwerveTypeConstants(
        double wheelDiameter, double angleGearRadio, double driveGearRadio, 
        double anglePIDF[], 
        InvertedValue angleMotorInverted, InvertedValue driveMotorInverted, InvertedValue canCoderInverted){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRadio = angleGearRadio;
        this.driveGearRadio = driveGearRadio;
        this.anglePIDF = anglePIDF;
        this.angleMotorInverted = angleMotorInverted;
        this.driveMotorInverted = driveMotorInverted;
        this.canCoderInverted = canCoderInverted;

    }
    public static SwerveTypeConstants SDSMK4I_L1(){
        double wheelDiameter = Conversions.inchesToMeters(4.0);

        double driveGearRadio = SDSMK4I_L2_DRIVEGEAR;
        double angleGearRadio = SDSMK4I_L2_ANGLEGEAR;

        InvertedValue driveMotorInverted = InvertedValue.Clockwise_Positive;
//        InvertedValue angleMotorInverted = InvertedValue.CounterClockwise_Positive;
        InvertedValue angleMotorInverted = InvertedValue.Clockwise_Positive;
        InvertedValue canCoderInverted = InvertedValue.Clockwise_Positive;

        double anglePIDF[] = {24, 0.0, 0.1};
//        double anglePIDF[] = {100.0, 0.0, 0.2};
        return new SwerveTypeConstants(
            wheelDiameter, angleGearRadio,driveGearRadio,
            anglePIDF, 
            angleMotorInverted, driveMotorInverted, canCoderInverted);
    }

    public static final double SDSMK4I_L1_DRIVEGEAR = 8.14;
    public static final double SDSMK4I_L1_ANGLEGEAR = 150.0/7.0;

    public static final double SDSMK4I_L2_DRIVEGEAR = 8.14;
    public static final double SDSMK4I_L2_ANGLEGEAR = 150.0/7.0;
}
