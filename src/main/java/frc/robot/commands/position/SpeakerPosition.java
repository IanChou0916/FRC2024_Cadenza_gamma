package frc.robot.commands.position;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubSystem;
import static frc.robot.Constants.ArmConstants.*;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.SPEAKER;

public class SpeakerPosition extends SequentialCommandGroup {
    private final ArmSubSystem armSubSystem;
    private final XboxController operatorController;
    private ARM_POSITIONS presentPosition;
    private double xOffset;

    public SpeakerPosition(ArmSubSystem armSubSystem, XboxController operatorController, ARM_POSITIONS presentPosition){
        this.armSubSystem = armSubSystem;
        this.operatorController = operatorController;
        this.presentPosition = presentPosition;
        addRequirements(armSubSystem);

        switch (presentPosition){
            case COLLECT -> addCommands(
                    new InstantCommand(()->armSubSystem.setArmPosition(SPEAKER)),
                    new InstantCommand(() -> armSubSystem.setArmPosition(SPEAKER)),
                    new WaitUntilCommand(()-> (SPEAKER.getArmPosition() - armSubSystem.getArmAngle() < 5)),
                    new InstantCommand(()-> armSubSystem.setWristPosition(SPEAKER)),
                    new WaitUntilCommand(()-> Math.abs(SPEAKER.getWristPosition() - armSubSystem.getWristAngle() ) < 4),
                    noticeSpeakerPosition()
            );
            case SPEAKER -> addCommands(
                    noticeSpeakerPosition()
            );
            case AMP -> addCommands(
                    new InstantCommand(() -> armSubSystem.setWristPosition(SPEAKER)),
                    new WaitUntilCommand(()->Math.abs(SPEAKER.getWristPosition() - armSubSystem.getWristAngle() )< 5),
                    new InstantCommand(()-> armSubSystem.setArmPosition(SPEAKER)),
                    new WaitUntilCommand(()-> Math.abs(SPEAKER.getArmPosition() - armSubSystem.getArmAngle() )< 5),
                    noticeSpeakerPosition()
            );
            default -> {
                break;
            }
        }
    }
    private SequentialCommandGroup noticeSpeakerPosition(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 0.5)),
                new WaitCommand(0.4),
                new InstantCommand(()->operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0)),
                new PrintCommand("Speaker Position Complete")
        );
    }

}
