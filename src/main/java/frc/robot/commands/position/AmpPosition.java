package frc.robot.commands.position;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubSystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.AMP;

public class AmpPosition extends SequentialCommandGroup {
    private final ArmSubSystem armSubSystem;
    private final XboxController operatorController;
    private ARM_POSITIONS presentPosition;
    private double xOffset;

    public AmpPosition(ArmSubSystem armSubSystem, XboxController operatorController, ARM_POSITIONS presentPosition) {
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
                    new WaitUntilCommand(() -> (AMP.getArmPosition() - armSubSystem.getArmAngle() < 10)),
                    new InstantCommand(() -> armSubSystem.setWristPosition(AMP)),
                    new WaitUntilCommand(() -> AMP.getWristPosition() - armSubSystem.getWristAngle() < 4),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                    new WaitCommand(0.4),
                    new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0)),
                    new PrintCommand("AMP Position Complete")
            );
            case COLLECT -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(AMP)),
                    new WaitUntilCommand(() -> (AMP.getArmPosition() - armSubSystem.getArmAngle() < 12)),
                    new InstantCommand(() -> armSubSystem.setWristPosition(AMP)),
                    new WaitUntilCommand(() -> AMP.getWristPosition() - armSubSystem.getWristAngle() < 5),
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

