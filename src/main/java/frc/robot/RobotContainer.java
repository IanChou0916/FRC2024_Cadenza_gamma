
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.ArmConstants.ARM_POSITIONS;
import frc.robot.Constants.ShootConstants.SHOOT_POSITIONS;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.CollectShootCommands;
import frc.robot.commands.HangCommands;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.auton.CollectCommand;
import frc.robot.commands.auton.PreloadCommand;
import frc.robot.commands.auton.ShootNote;
import frc.robot.commands.position.CollectPosition;
import frc.robot.commands.position.SpeakerPosition;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.*;



import static edu.wpi.first.wpilibj.XboxController.Button.*;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.*;
import static frc.robot.Constants.LEDConstants.NORMAL_POSITION_COLOR;
import static frc.robot.Constants.ShootConstants.SHOOT_POSITIONS.*;
import static frc.robot.RobotMap.Vision.LIMELIGHT_ALIGNMENT;


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

  private final Field2d field = new Field2d();
  private SendableChooser <Command> autoChooser;
  private boolean hangFinished;


  public RobotContainer() {
    // This is the field that will be displayed on the SmartDashboard

    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);

    setRegisterCommand();

    autoChooser = AutoBuilder.buildAutoChooser();


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


    /**
     * 
     * 
    collectSubSystem.setDefaultCommand(new CollectShootCommands(
            collectSubSystem,
            shootSubSystem,
            ledSubsystem,
            operatorController::getPOV,
            operatorController::getLeftTriggerAxis
    ));
    **/
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



    new JoystickButton(operatorController,kX.value)
            .onTrue(positionManager.TargetAmpPosition());

    new JoystickButton(operatorController,kY.value)
            .onTrue(positionManager.TargetCollectPosition());

    new JoystickButton(operatorController,kB.value)
            .onTrue(positionManager.TargetSpeakerPosition());

    new JoystickButton(operatorController,kA.value)
            .onTrue(positionManager.TargetNormalPosition());
    new POVButton(operatorController, 0)
            .onTrue(new ShootSequence(collectSubSystem,shootSubSystem,ledSubsystem,SHOOT_POSITIONS.SPEAKER));
    new POVButton(operatorController, 180)
            .onTrue(new ShootSequence(collectSubSystem,shootSubSystem,ledSubsystem,SHOOT_POSITIONS.COLLECT));
    new POVButton(operatorController, 90)
            .onTrue(new ShootSequence(collectSubSystem,shootSubSystem,ledSubsystem,SHOOT_POSITIONS.AMP));

  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
  private void selectAuto(){

    autoChooser.setDefaultOption("Mid Leave",AutoBuilder.buildAuto("MID_LEAVE"));

    //autoChooser.addOption("BACK LEAVE",AutoBuilder.buildAuto("BACK_LEAVE"));
    //NamedCommands.getCommand("Preload");
  }
  private void setRegisterCommand(){
    // Register Commands

    NamedCommands.registerCommand("Preload", new PreloadCommand(armSubSystem,shootSubSystem,collectSubSystem));
    NamedCommands.registerCommand("Collect",new CollectCommand(shootSubSystem,collectSubSystem));
    NamedCommands.registerCommand("CollectPosition",new CollectPosition(armSubSystem,operatorController,ARM_POSITIONS.SPEAKER));
    NamedCommands.registerCommand("ShootNote",new ShootNote(shootSubSystem,collectSubSystem));
    NamedCommands.registerCommand("ToSpeaker",new SpeakerPosition(armSubSystem,operatorController,ARM_POSITIONS.COLLECT));
    NamedCommands.registerCommand("Start",new PrintCommand("Auto Start."));
    NamedCommands.registerCommand("setAmpPosition", new PrintCommand("AMP"));

    // Get Commands

    NamedCommands.getCommand("Preload");
    NamedCommands.getCommand("Collect");
    NamedCommands.getCommand("CollectPosition");
    NamedCommands.getCommand("ToSpeaker");
    NamedCommands.getCommand("ShootNote");
    NamedCommands.getCommand("ToSpeaker");


  }
  public void disableInit(){
    disableLimelightLED();
    operatorController.setRumble(GenericHID.RumbleType.kBothRumble,0);
  }
  public void disablePeriodic(){
    disableLimelightLED();
    ledSubsystem.rainbow();
    ledSubsystem.write();
  }

  public void autonomousInit(){
    enableLimeLightLED();
    shootSubSystem.stopShoot();
    collectSubSystem.stopCollect();
  }
  public void teleopInit(){
    enableLimeLightLED();
    swerveSubSystem.resetModulesToAbsolute();
    ledSubsystem.fillRGB(NORMAL_POSITION_COLOR);

  }

  private void enableLimeLightLED(){
   LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_ALIGNMENT);
  }
  private void disableLimelightLED(){
    LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_ALIGNMENT);
  }
}



