package frc.robot.commands.position;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.NORMAL;
import static frc.robot.Constants.JoyStickConstants.NORMAL_RUMBLE;
import static frc.robot.Constants.JoyStickConstants.NOTICE_VALUE;

public class NormalPosition extends SequentialCommandGroup {
    private final ArmSubsystem armSubSystem;
    private final XboxController operatorController;
    private ARM_POSITIONS presentPosition;
    private double xOffset;

    public NormalPosition(ArmSubsystem armSubSystem, XboxController operatorController, ARM_POSITIONS presentPosition){
        this.armSubSystem = armSubSystem;
        this.operatorController = operatorController;
        this.presentPosition = presentPosition;
        addRequirements(armSubSystem);

        switch (presentPosition){
            case NORMAL -> addCommands(
                    noticeNormalCompleted()
            );
            case SPEAKER -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(NORMAL)),
                    new WaitUntilCommand(()-> (NORMAL.getArmPosition() - armSubSystem.getArmAngle()) < 5),
                    new InstantCommand(()-> armSubSystem.setWristPosition(NORMAL)),
                    new WaitUntilCommand(()-> (NORMAL.getWristPosition() - armSubSystem.getWristAngle() ) < 3),
                    noticeNormalCompleted()
            );
            case AMP -> addCommands(
                    new InstantCommand(() -> armSubSystem.setArmPosition(NORMAL.getArmPosition())),
                    new WaitUntilCommand(()-> (armSubSystem.getArmAngle() - NORMAL.getArmPosition()) < 15),
                    new InstantCommand(()-> armSubSystem.setWristPosition(NORMAL)),
                    new WaitUntilCommand(()-> (NORMAL.getWristPosition() - armSubSystem.getWristAngle() )< 3),
                    noticeNormalCompleted()
            );
            case COLLECT -> addCommands(
                    new InstantCommand(()-> armSubSystem.setWristPosition(NORMAL)),
                    new WaitUntilCommand(()-> (NORMAL.getWristPosition() - armSubSystem.getWristAngle() )< 4),
                    new InstantCommand(() -> armSubSystem.setArmPosition(NORMAL.getArmPosition())),
                    new WaitUntilCommand(()-> (armSubSystem.getArmAngle() - NORMAL.getArmPosition()) < 4),
                    noticeNormalCompleted()
            );
            default -> {
                break;
            }
        }
    }
    private SequentialCommandGroup noticeNormalCompleted(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> operatorController.setRumble(NORMAL_RUMBLE, NOTICE_VALUE)),
                new WaitCommand(0.4),
                new InstantCommand(()->operatorController.setRumble(NORMAL_RUMBLE,0)),
                new PrintCommand("NORMAL Position Complete")
        );
    }

}
