// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.CollectShootCommands;
import frc.robot.commands.HangCommands;
import frc.robot.commands.auton.PreloadCommand;
import frc.robot.subsystems.HangSubSystem;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.*;

import static edu.wpi.first.wpilibj.Joystick.AxisType.kX;
import static edu.wpi.first.wpilibj.PS5Controller.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static edu.wpi.first.wpilibj.XboxController.*;

import java.util.List;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.NORMAL;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.SPEAKER;
import static frc.robot.Constants.AutoConstants.SwervePathFollower;


public class RobotContainer {
  private final XboxController driveController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final SwerveSubSystem swerveSubSystem = new SwerveSubSystem();
  private final CollectSubSystem collectSubSystem = new CollectSubSystem();
  private final ShootSubSystem shootSubSystem = new ShootSubSystem();
  private final ArmSubSystem armSubSystem = new ArmSubSystem();
  private final HangSubSystem hangSubSystem = new HangSubSystem();
  private PositionManager positionManager = new PositionManager(armSubSystem,collectSubSystem,operatorController,NORMAL);

  private final Field2d field;
  private final SendableChooser <Command> autoChooser;
  private boolean hangFinished;


  public RobotContainer() {
    // This is the field that will be displayed on the SmartDashboard

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    AutoBuilder.configureHolonomic(
            swerveSubSystem::getPose,
            swerveSubSystem::resetOdometry,
            swerveSubSystem::getChassisSpeeds,
            swerveSubSystem::setChassisSpeeds,
            SwervePathFollower,
            ()-> {
              var alliance = DriverStation.getAlliance();
              if(alliance.isPresent()){
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            swerveSubSystem
    );
    autoChooser = AutoBuilder.buildAutoChooser();
    NamedCommands.registerCommand("TestCommand",new PrintCommand("Hello World!"));
    NamedCommands.registerCommand("setAmpPosition",new PrintCommand("AMP"));
    NamedCommands.registerCommand("Preload", new PreloadCommand(armSubSystem,shootSubSystem,collectSubSystem));

    SmartDashboard.putData("Auto Mode", autoChooser);
    selectAuto();

    swerveSubSystem.setDefaultCommand(new SwerveDriveCommand(
            swerveSubSystem,
            driveController::getLeftY,
            driveController::getLeftX,
            driveController::getRightX,
            driveController::getPOV,
            driveController::getLeftBumper,
            driveController::getRightTriggerAxis)); // Vision Detect.


    collectSubSystem.setDefaultCommand(new CollectShootCommands(
            collectSubSystem,
            shootSubSystem,
            operatorController::getPOV
    ));

    armSubSystem.setDefaultCommand(new ArmCommands(
            armSubSystem,
            operatorController::getStartButton,
            operatorController::getBackButton,
            operatorController::getLeftBumper,
            operatorController::getRightBumper
    ));



    hangSubSystem.setDefaultCommand(new HangCommands(
            hangSubSystem,
            driveController::getXButton, // Up Left
            driveController::getBButton, // Down Left
            driveController::getStartButton, // Up Right
            driveController::getBackButton // Down Right.
    ));

    configureBindings();
  }

  public void robotInit(){
    //swerveSubSystem.zeroGyro();
  }

  private void configureBindings() {
    new JoystickButton(driveController, kRightBumper.value) // Right Bumper
           .onTrue(new InstantCommand(swerveSubSystem::zeroGyro));

    new JoystickButton(operatorController,kX.value)
            .onTrue(positionManager.TargetAmpPosition());

    new JoystickButton(operatorController,kY.value)
            .onTrue(positionManager.TargetCollectPosition());

    new JoystickButton(operatorController,kB.value)
            .onTrue(positionManager.TargetSpeakerPosition());

    new JoystickButton(operatorController,kA.value)
            .onTrue(positionManager.TargetNormalPosition());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  private void selectAuto(){

    autoChooser.setDefaultOption("Mid Leave",AutoBuilder.buildAuto("MID_LEAVE"));
    autoChooser.addOption("BACK LEAVE",AutoBuilder.buildAuto("BACK_LEAVE"));
    NamedCommands.getCommand("Preload");


  }
}


  

