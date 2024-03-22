package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.PositionManager;
import frc.robot.commands.position.CollectPosition;
import frc.robot.commands.position.NormalPosition;
import frc.robot.commands.position.SpeakerPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.NORMAL;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.SPEAKER;

public class PreloadCommand extends SequentialCommandGroup {
    public PreloadCommand(ArmSubsystem armSubSystem, ShootSubsystem shootSubSystem, CollectSubsystem collectSubSystem) {
        this.addRequirements(armSubSystem);
        this.addRequirements(shootSubSystem);
        this.addRequirements(collectSubSystem);
        addCommands(
                new PrintCommand("Start"),
                new InstantCommand(() -> armSubSystem.setWristPosition(SPEAKER)),
                new WaitUntilCommand(()->Math.abs(SPEAKER.getWristPosition() - armSubSystem.getWristAngle() )< 6),
                new InstantCommand(()-> armSubSystem.setArmPosition(SPEAKER)),
                new WaitCommand(0.4),
                new InstantCommand(shootSubSystem::shootNote),
                new WaitCommand(1.25
                ),
                new InstantCommand(collectSubSystem::collectNote),
                new WaitCommand(1.0),
                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::stopShoot),
                        new InstantCommand(collectSubSystem::stopCollect)
                )
                //new NormalPosition(armSubSystem,null,SPEAKER)
        );

    }
}
