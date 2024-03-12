package frc.robot.commands.position;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubSystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.COLLECT;
import static frc.robot.Constants.JoyStickConstants.*;

public class CollectPosition extends SequentialCommandGroup {
    private final ArmSubSystem armSubSystem;
    private final XboxController operatorController;
    private ARM_POSITIONS presentPosition;
    private double xOffset;

    public CollectPosition(ArmSubSystem armSubSystem, XboxController operatorController, ARM_POSITIONS presentPosition){
        this.armSubSystem = armSubSystem;
        this.operatorController = operatorController;
        this.presentPosition = presentPosition;
        addRequirements(armSubSystem);

        switch (presentPosition){
            case COLLECT -> addCommands(
                    new InstantCommand(() -> operatorController.setRumble(COLLECT_RUMBLE, NOTICE_VALUE)),
                    new WaitCommand(0.4),
                    new InstantCommand(()->operatorController.setRumble(COLLECT_RUMBLE,0)),
                    new PrintCommand("COLLECT Position Complete")
            );
            case SPEAKER -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(COLLECT)),
                    new WaitUntilCommand(()-> (COLLECT.getArmPosition() - armSubSystem.getArmAngle() < 5)),
                    new InstantCommand(()-> armSubSystem.setWristPosition(COLLECT)),
                    new WaitUntilCommand(()-> Math.abs(COLLECT.getWristPosition() - armSubSystem.getWristAngle() ) < 3),
                    new InstantCommand(() -> operatorController.setRumble(COLLECT_RUMBLE, NOTICE_VALUE)),
                    new WaitCommand(0.4),
                    new InstantCommand(()->operatorController.setRumble(COLLECT_RUMBLE,0)),
                    new PrintCommand("COLLECT Position Complete")
            );
            case AMP -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(COLLECT.getArmPosition())),
                    new WaitUntilCommand(()-> (armSubSystem.getArmAngle() - COLLECT.getArmPosition() < 15)),
                    new InstantCommand(()-> armSubSystem.setWristPosition(COLLECT)),
                    new WaitUntilCommand(()-> Math.abs(COLLECT.getWristPosition() - armSubSystem.getWristAngle() )< 3),
                    new InstantCommand(() -> operatorController.setRumble(COLLECT_RUMBLE, NOTICE_VALUE)),
                    new WaitCommand(0.4),
                    new InstantCommand(()->operatorController.setRumble(COLLECT_RUMBLE,0)),
                    new PrintCommand("COLLECT Position Complete")
            );
            default -> {
                break;
            }
        }
    }
    private SequentialCommandGroup noticeCollectCompleted(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> operatorController.setRumble(COLLECT_RUMBLE, NOTICE_VALUE)),
                new WaitCommand(0.4),
                new InstantCommand(()->operatorController.setRumble(COLLECT_RUMBLE,0)),
                new PrintCommand("COLLECT Position Complete")
        );
    }

}
