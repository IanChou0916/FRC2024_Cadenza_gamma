package frc.robot.commands.position;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubsystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.AMP;

public class AmpPosition extends SequentialCommandGroup {
    private final ArmSubsystem armSubSystem;
    private final XboxController operatorController;
    private ARM_POSITIONS presentPosition;
    private double xOffset;

    public AmpPosition(ArmSubsystem armSubSystem, XboxController operatorController, ARM_POSITIONS presentPosition) {
        this.armSubSystem = armSubSystem;
        this.operatorController = operatorController;
        this.presentPosition = presentPosition;
        addRequirements(armSubSystem);

        switch (presentPosition) {
            case AMP -> addCommands(
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                    new WaitCommand(0.4),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0)),
                    new PrintCommand("AMP Position Complete")
            );
            case SPEAKER -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(AMP)),
                    new WaitUntilCommand(() -> (AMP.getArmPosition() - armSubSystem.getArmAngle()) < 6),
                    new InstantCommand(() -> armSubSystem.setWristPosition(AMP)),
                    new WaitUntilCommand(() -> Math.abs(AMP.getWristPosition() - armSubSystem.getWristAngle()) < 4),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                    new WaitCommand(0.4),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0)),
                    new PrintCommand("AMP Position Complete")
            );
            case COLLECT -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(AMP)),
                    new WaitUntilCommand(() -> (AMP.getArmPosition() - armSubSystem.getArmAngle() < 7)),
                    new InstantCommand(() -> armSubSystem.setWristPosition(AMP)),
                    new WaitUntilCommand(() -> Math.abs(AMP.getWristPosition() - armSubSystem.getWristAngle()) < 5),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                    new WaitCommand(0.4),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0)),
                    new PrintCommand("AMP Position Complete")
            );
            case NORMAL -> addCommands(
                    new InstantCommand(() -> armSubSystem.setWristPosition(AMP)),
                    new WaitUntilCommand(() ->Math.abs(AMP.getWristPosition() - armSubSystem.getWristAngle() )< 6),
                    new InstantCommand(() -> armSubSystem.setArmPosition(AMP)),
                    new WaitUntilCommand(() -> (AMP.getArmPosition() - armSubSystem.getArmAngle() )< 5),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                    new WaitCommand(0.4),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0)),
                    new PrintCommand("AMP Position Complete")
            );
            default -> {
                break;
            }
        }
    }
    private SequentialCommandGroup noticeAmpComplete(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                new WaitCommand(0.4),
                new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0)),
                new PrintCommand("AMP Position Complete")
        );
    };
}

