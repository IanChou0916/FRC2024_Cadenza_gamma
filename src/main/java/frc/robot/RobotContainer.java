// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ArmTestCommands;
import frc.robot.commands.CollectShootCommands;
import frc.robot.commands.HangSubSystem;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.*;


public class RobotContainer {
  private final XboxController driveController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final SwerveSubSystem swerveSubSystem = new SwerveSubSystem();
  private final CollectSubSystem collectSubSystem = new CollectSubSystem();
  private final ShootSubSystem shootSubSystem = new ShootSubSystem();
  private final ArmSubSystem armSubSystem = new ArmSubSystem();
  private final HangSubSystem hangSubSystem = new HangSubSystem();


  private final Field2d field;
  private final SendableChooser <Command> autoChooser;
  public RobotContainer() {
    configureBindings();

    field = new Field2d();  // This is the field that will be displayed on the SmartDashboard
    SmartDashboard.putData("Field", field);

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
      /*
      armSubSystem.setDefaultCommand(new ArmCommands(
              armSubSystem,operatorController::getLeftY,
              operatorController::getRightY
        ));

       */
    armSubSystem.setDefaultCommand(new ArmTestCommands(
            armSubSystem,
            operatorController::getBButton,
            operatorController::getAButton,
            operatorController::getLeftBumper,
            operatorController::getRightBumper
    ));
        
    autoChooser = AutoBuilder.buildAutoChooser();
    NamedCommands.registerCommand("TestCommand",new PrintCommand("Hello World!"));
    NamedCommands.registerCommand("setAmpPosition",new PrintCommand("AMP"));
    SmartDashboard.putData("Auto Mode", autoChooser);
    selectAuto();

  }

  public void robotInit(){
    //swerveSubSystem.zeroGyro();
  }

  private void configureBindings() {
    new JoystickButton(driveController, 6) // Right Bumper
           .onTrue(new InstantCommand(swerveSubSystem::zeroGyro));

    new JoystickButton(driveController,XboxController.Button.kX.value)
            .onTrue(new InstantCommand(hangSubSystem::setLeftHangMotor));
    new JoystickButton(driveController,XboxController.Button.kY.value)
            .onTrue(new InstantCommand(()->{
              hangSubSystem.stopLeftHangMotor();
              hangSubSystem.stopRightHangMotor();
            }));
    new JoystickButton(driveController,XboxController.Button.kB.value)
            .onTrue(new InstantCommand(hangSubSystem::reverseLeftHangMotor));
    new JoystickButton(driveController,XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(hangSubSystem::setRightHangMotor));
    new JoystickButton(driveController,XboxController.Button.kBack.value)
            .onTrue(new InstantCommand(hangSubSystem::reverseRightHangMotor));





  }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }
  private void selectAuto(){

    autoChooser.setDefaultOption("BACK LEAVE",AutoBuilder.buildAuto("BACK_LEAVE"));
    autoChooser.addOption("Mid Leave",AutoBuilder.buildAuto("MID_LEAVE"));
  }
}

  

