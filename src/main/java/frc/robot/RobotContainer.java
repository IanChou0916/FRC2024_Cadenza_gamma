
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.CollectShootCommands;
import frc.robot.commands.HangCommands;
import frc.robot.commands.auton.PreloadCommand;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.*;


import static edu.wpi.first.wpilibj.XboxController.Button.*;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.NORMAL;



public class RobotContainer {
  private final XboxController driveController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final SwerveSubsystem swerveSubSystem = new SwerveSubsystem();
  private final CollectSubsystem collectSubSystem = new CollectSubsystem();
  private final ShootSubsystem shootSubSystem = new ShootSubsystem();
  private final ArmSubsystem armSubSystem = new ArmSubsystem();
  private final HangSubsystem hangSubSystem = new HangSubsystem();
  private final LedSubsystem ledSubsystem = new LedSubsystem();
  private PositionManager positionManager = new PositionManager(armSubSystem,collectSubSystem,operatorController,NORMAL);

  private final Field2d field;
  private SendableChooser <Command> autoChooser;
  private boolean hangFinished;


  public RobotContainer() {
    // This is the field that will be displayed on the SmartDashboard

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    autoChooser = AutoBuilder.buildAutoChooser();
    //NamedCommands.registerCommand("TestCommand",new PrintCommand("Hello World!"));
    //NamedCommands.registerCommand("setAmpPosition",new PrintCommand("AMP"));
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
            ledSubsystem,
            operatorController::getPOV
    ));

    /*

    collectSubSystem.setDefaultCommand(new CollectShooTestCommands(
            collectSubSystem,
            shootSubSystem,
            armSubSystem,
            operatorController::getPOV
    ));

     */



    armSubSystem.setDefaultCommand(new ArmCommand(
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
    new JoystickButton(driveController, kA.value) // Right Bumper
            .onTrue(new InstantCommand(swerveSubSystem::zeroGyro));

    new JoystickButton(driveController, kRightBumper.value)
            .whileTrue(new InstantCommand(()->{
              shootSubSystem.reverseNote();
              collectSubSystem.reverseCollect();
            }));

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

    //autoChooser.addOption("BACK LEAVE",AutoBuilder.buildAuto("BACK_LEAVE"));
    //NamedCommands.getCommand("Preload");
  }
  public void disableInit(){
    operatorController.setRumble(GenericHID.RumbleType.kBothRumble,0);
  }
  public void disablePeriodic(){
    ledSubsystem.rainbow();
    ledSubsystem.write();
  }
}



