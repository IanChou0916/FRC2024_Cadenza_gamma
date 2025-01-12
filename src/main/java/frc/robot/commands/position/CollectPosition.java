package frc.robot.commands.position;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.COLLECT;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.NORMAL;
import static frc.robot.Constants.JoyStickConstants.*;

public class CollectPosition extends SequentialCommandGroup {
    private final ArmSubsystem armSubSystem;
    private final XboxController operatorController;

    private ARM_POSITIONS presentPosition;
    private double xOffset;

    public CollectPosition(ArmSubsystem armSubSystem, XboxController operatorController, ARM_POSITIONS presentPosition){
        this.armSubSystem = armSubSystem;
        this.operatorController = operatorController;
        this.presentPosition = presentPosition;
        addRequirements(armSubSystem);

        switch (presentPosition){
            case COLLECT -> addCommands(
                    noticeCollectCompleted()
            );
            case SPEAKER -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(COLLECT)),
                    new WaitUntilCommand(()-> (COLLECT.getArmPosition() - armSubSystem.getArmAngle() < 5)),
                    new InstantCommand(()-> armSubSystem.setWristPosition(COLLECT)),
                    new WaitUntilCommand(()-> (COLLECT.getWristPosition() - armSubSystem.getWristAngle() ) < 5),
                    noticeCollectCompleted()
            );
            case AMP -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(COLLECT.getArmPosition())),
                    new WaitUntilCommand(()-> (armSubSystem.getArmAngle() - COLLECT.getArmPosition() < 5)),
                    new InstantCommand(()-> armSubSystem.setWristPosition(COLLECT)),
                    new WaitUntilCommand(()-> (COLLECT.getWristPosition() - armSubSystem.getWristAngle() )< 5),
                    noticeCollectCompleted()
            );
            case NORMAL -> addCommands(
                    new InstantCommand(()-> armSubSystem.setWristPosition(COLLECT)),
                    new WaitUntilCommand(()-> (COLLECT.getWristPosition() - armSubSystem.getWristAngle() )< 20),
                    new WaitCommand(0.25),
                    new InstantCommand(() -> armSubSystem.setArmPosition(COLLECT.getArmPosition())),
                    new WaitUntilCommand(()-> (armSubSystem.getArmAngle() - COLLECT.getArmPosition()) < 5),
                    noticeCollectCompleted()
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
