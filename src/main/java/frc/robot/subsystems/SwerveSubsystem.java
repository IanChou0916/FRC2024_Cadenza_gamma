package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveTypeConstants;
import frc.robot.Constants;

import static frc.robot.Constants.AutoConstants.SwervePathFollower;
import static frc.robot.RobotMap.*;

import static frc.robot.Constants.NAVX_INVERTED;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.RobotMap.SwerveMap.*;


public class SwerveSubsystem extends SubsystemBase {

    public swerveModule[] swerveModules;
    public SwerveDriveKinematics swerveDriveKinematics = L2_SwerveDriveKinematics;
    public SwerveDriveOdometry swerveDriveOdometry ;
    public AHRS navX;

    public SwerveSubsystem(){
        navX = new AHRS(SPI.Port.kMXP, Constants.NAVX_UPDATE_RATE);
        navX.reset();

        swerveModules = new swerveModule[]{
                new swerveModule(
                        0,SwerveTypeConstants.SDSMK4I_L2(),
                        FRONT_LEFT_DRIVE_MOTOR_ID,
                        FRONT_LEFT_ANGLE_MOTOR_ID,
                        FRONT_LEFT_CANCODER_MOTOR_ID,
                        FRONT_LEFT_ANGLE_OFFSET),
                new swerveModule(
                        1,SwerveTypeConstants.SDSMK4I_L2(),
                        BACK_LEFT_DRIVE_MOTOR_ID,
                        BACK_LEFT_ANGLE_MOTOR_ID,
                        BACK_LEFT_CANCODER_MOTOR_ID,
                        BACK_LEFT_ANGLE_OFFSET),
                new swerveModule(
                        2,SwerveTypeConstants.SDSMK4I_L2(),
                        BACK_RIGHT_DRIVE_MOTOR_ID,
                        BACK_RIGHT_ANGLE_MOTOR_ID,
                        BACK_RIGHT_CANCODER_MOTOR_ID,
                        BACK_RIGHT_ANGLE_OFFSET),
                new swerveModule(
                        3,SwerveTypeConstants.SDSMK4I_L2(),
                        FRONT_RIGHT_DRIVE_MOTOR_ID,
                        FRONT_RIGHT_ANGLE_MOTOR_ID,
                        FRONT_RIGHT_CANCODER_MOTOR_ID,
                        FRONT_RIGHT_ANGLE_OFFSET),
        };
        swerveDriveKinematics = L2_SwerveDriveKinematics;
        swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, navX.getRotation2d(), getModulePositions());

        Timer.delay(0.5);
        resetModulesToAbsolute();


        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::setChassisSpeeds,
                SwervePathFollower,
                ()-> {
                    var alliance = DriverStation.getAlliance();
                    if(alliance.isPresent()){
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );





    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
        SwerveModuleState[] swerveModuleStates =
                swerveDriveKinematics.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getYaw())
                                : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        L2_SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,SWERVE_MAX_SPEED);
        for (swerveModule module : swerveModules){
            module.setDesireState(swerveModuleStates[module.moduleNumber]);
        }
    }
    // AutoBuilder
    public Pose2d getPose(){
        return swerveDriveOdometry.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose){
        swerveDriveOdometry.resetPosition(getYaw(),getModulePositions(),pose);
    }
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDriveKinematics.toChassisSpeeds(getModuleStates());
    }
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds,0.02);
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        //SmartDashboard.putNumber("TEST", swerveModuleStates[1].speedMetersPerSecond)
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates){

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SWERVE_MAX_SPEED);

        for (swerveModule module : swerveModules){
            module.setDesireState(swerveModuleStates[module.moduleNumber]);
        }
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (swerveModule module : swerveModules){
            modulePositions[module.moduleNumber] = module.getPosition();
        }

        return modulePositions;
    }
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (swerveModule module : swerveModules){
            states[module.moduleNumber] = module.getState();
        }

        return states;

    }

    public Rotation2d getYaw(){
        return (NAVX_INVERTED) ? Rotation2d.fromDegrees(360 - navX.getYaw()) : Rotation2d.fromDegrees(navX.getYaw());
    }
    public void setGyroAngle(double angle){
        navX.setAngleAdjustment(angle);
    }
    public void resetModulesToAbsolute(){
        for(swerveModule module : swerveModules){
            module.resetToAbosolute();
        }
    }
    public void zeroGyro(){
        navX.reset();
    }
    public void stopAll(){
        for (swerveModule swerveModule : swerveModules) {
            swerveModule.stop();
        }
    }


    @Override
    public void periodic() {

        swerveDriveOdometry.update(getYaw(), getModulePositions());
        /*
        for(swerveModule module : swerveModules){
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " CanCoder",module.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " Integrated", module.getPosition().angle.getDegrees()%360);
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " POS", module.getPosition().distanceMeters);
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " State", module.getState().angle.getDegrees());
        }

        SmartDashboard.putString("O", swerveDriveOdometry.getPoseMeters().toString());


         */
        // TODO : Tuning Completed.
        SmartDashboard.putNumber("Rotation",navX.getYaw());
    }

}
