package frc.robot.commands.position;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubSystem;
import frc.robot.subsystems.CollectSubSystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.AMP;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.COLLECT;

public class PositionManager {
    private final ArmSubSystem armSubSystem;
    private final CollectSubSystem collectSubSystem;
    private final XboxController operatorController;
    private ARM_POSITIONS presentPosition;
    private ARM_POSITIONS targetPosition;

    public PositionManager(
            ArmSubSystem armSubSystem,
            CollectSubSystem collectSubSystem,
            XboxController operatorController,
            ARM_POSITIONS presentPosition){
        this.armSubSystem = armSubSystem;
        this.collectSubSystem = collectSubSystem;
        this.operatorController = operatorController;
        this.presentPosition = presentPosition;
    }
    public void setTargetPosition(ARM_POSITIONS Position){
        targetPosition = Position;
    }
    public void setPresentPosition(ARM_POSITIONS Position){
        presentPosition = Position;
    }


    public SequentialCommandGroup ToAmpPosition() {
        switch (presentPosition) {
            case AMP -> {
                return new SequentialCommandGroup(
                        new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                        new WaitCommand(0.4),
                        new InstantCommand(()->operatorController.setRumble(GenericHID.RumbleType.kLeftRumble,0)),
                        new PrintCommand("AMP Position Complete")
                );

            }
            case SPEAKER -> {
                setPresentPosition(AMP);
                return new SequentialCommandGroup(
                        new InstantCommand(() -> armSubSystem.setArmPosition(AMP)),
                        new WaitUntilCommand(()-> (AMP.getArmPosition() - armSubSystem.getArmAngle() < 10)),
                        new InstantCommand(()-> armSubSystem.setWristPosition(AMP)),
                        new WaitUntilCommand(()-> AMP.getWristPosition() - armSubSystem.getWristAngle() < 5),
                        new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                        new WaitCommand(0.4),
                        new InstantCommand(()->operatorController.setRumble(GenericHID.RumbleType.kLeftRumble,0)),
                        new PrintCommand("AMP Position Complete")
                );
            }
            case COLLECT -> {
                setPresentPosition(AMP);
                return new SequentialCommandGroup(
                        new InstantCommand(() -> armSubSystem.setArmPosition(AMP)),
                        new WaitUntilCommand(()-> (AMP.getArmPosition() - armSubSystem.getArmAngle() < 12)),
                        new InstantCommand(()-> armSubSystem.setWristPosition(AMP)),
                        new WaitUntilCommand(()-> AMP.getWristPosition() - armSubSystem.getWristAngle() < 3),
                        new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                        new WaitCommand(0.4),
                        new InstantCommand(()->operatorController.setRumble(GenericHID.RumbleType.kLeftRumble,0)),
                        new PrintCommand("AMP Position Complete")
                );
                //new WaitUntilCommand()

            }
            default -> {
                break;
            }
        }
        return new SequentialCommandGroup();
    }
    public SequentialCommandGroup ToCollectPosition() {
        switch (presentPosition) {
            case COLLECT -> {
                return new SequentialCommandGroup(
                        new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                        new WaitCommand(0.4),
                        new InstantCommand(()->operatorController.setRumble(GenericHID.RumbleType.kLeftRumble,0)),
                        new PrintCommand("Collect Position Complete")

                );

            }
            case SPEAKER -> {
                setPresentPosition(COLLECT);
                return new SequentialCommandGroup(
                        new InstantCommand(() -> armSubSystem.setArmPosition(COLLECT)),
                        new WaitUntilCommand(()-> (COLLECT.getArmPosition() - armSubSystem.getArmAngle() < 5)),
                        new InstantCommand(()-> armSubSystem.setWristPosition(COLLECT)),
                        new WaitUntilCommand(()-> Math.abs(COLLECT.getWristPosition() - armSubSystem.getWristAngle() ) < 3),
                        new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                        new WaitCommand(0.4),
                        new InstantCommand(()->operatorController.setRumble(GenericHID.RumbleType.kLeftRumble,0)),
                        new PrintCommand("AMP Position Complete")
                );
            }
            case AMP -> {
                setPresentPosition(COLLECT);
                return new SequentialCommandGroup(
//                        new InstantCommand(() -> armSubSystem.setArmPosition(AMP.getArmPosition()-20)),
                        new InstantCommand(() -> armSubSystem.setArmPosition(COLLECT.getArmPosition()+20)),
                        //new WaitUntilCommand(()-> (AMP.getArmPosition() - armSubSystem.getArmAngle() < 10)),
                        new WaitCommand(1),
                        new InstantCommand(() -> armSubSystem.setArmPosition(COLLECT.getArmPosition())),
                        new WaitUntilCommand(()-> (armSubSystem.getArmAngle() - COLLECT.getArmPosition() < 15)),
                        new WaitCommand(1.0),
                        new InstantCommand(()-> armSubSystem.setWristPosition(COLLECT)),
                        new WaitUntilCommand(()-> Math.abs(COLLECT.getWristPosition() - armSubSystem.getWristAngle() )< 5),
                        new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5)),
                        new WaitCommand(0.4),
                        new InstantCommand(()->operatorController.setRumble(GenericHID.RumbleType.kLeftRumble,0)),
                        new PrintCommand("AMP Position Complete")
                );
                //new WaitUntilCommand()

            }
            default -> {
                break;
            }
        }
        return new SequentialCommandGroup();
    }
}
