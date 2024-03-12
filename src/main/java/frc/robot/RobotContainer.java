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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.CollectShootCommands;
import frc.robot.commands.HangCommands;
import frc.robot.subsystems.HangSubSystem;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.*;

import java.util.List;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.SPEAKER;


public class RobotContainer {
  private final XboxController driveController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final SwerveSubSystem swerveSubSystem = new SwerveSubSystem();
  private final CollectSubSystem collectSubSystem = new CollectSubSystem();
  private final ShootSubSystem shootSubSystem = new ShootSubSystem();
  private final ArmSubSystem armSubSystem = new ArmSubSystem();
  private final HangSubSystem hangSubSystem = new HangSubSystem();
  private PositionManager positionManager = new PositionManager(armSubSystem,collectSubSystem,operatorController,SPEAKER);

  private final Field2d field;
  private final SendableChooser <Command> autoChooser;


  public RobotContainer() {
    // This is the field that will be displayed on the SmartDashboard

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    autoChooser = AutoBuilder.buildAutoChooser();
    NamedCommands.registerCommand("TestCommand",new PrintCommand("Hello World!"));
    NamedCommands.registerCommand("setAmpPosition",new PrintCommand("AMP"));
    SmartDashboard.putData("Auto Mode", autoChooser);
    selectAuto();

    swerveSubSystem.setDefaultCommand(new SwerveDriveCommand(
            swerveSubSystem,
            () -> driveController.getRawAxis(XboxController.Axis.kLeftY.value),
            () -> driveController.getRawAxis(XboxController.Axis.kLeftX.value),
            () -> driveController.getRawAxis(XboxController.Axis.kRightX.value),
            driveController::getPOV,
            driveController::getLeftBumper,
            driveController::getAButton));


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
            driveController::getXButton,
            driveController::getBButton,
            driveController::getStartButton,
            driveController::getBackButton
    ));

    configureBindings();
  }

  public void robotInit(){
    swerveSubSystem.zeroGyro();
  }

  private void configureBindings() {
    new JoystickButton(driveController, XboxController.Button.kRightBumper.value) // Right Bumper
           .onTrue(new InstantCommand(swerveSubSystem::zeroGyro));
    new JoystickButton(operatorController,XboxController.Button.kX.value)
            .onTrue(positionManager.TargetAmpPosition());
    new JoystickButton(operatorController,XboxController.Button.kA.value)
            .onTrue(positionManager.TargetCollectPosition());
    new JoystickButton(operatorController,XboxController.Button.kB.value)
            .onTrue(positionManager.TargetSpeakerPosition());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  private void selectAuto(){

    autoChooser.setDefaultOption("BACK LEAVE",AutoBuilder.buildAuto("BACK_LEAVE"));
    autoChooser.addOption("Mid Leave",AutoBuilder.buildAuto("MID_LEAVE"));
  }
}


  

