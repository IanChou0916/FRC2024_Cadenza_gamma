package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ShootNote extends SequentialCommandGroup {
    public ShootNote(ShootSubsystem shootSubSystem, CollectSubsystem collectSubSystem) {
        this.addRequirements(shootSubSystem);
        this.addRequirements(collectSubSystem);
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::reverseNote),
                        new InstantCommand(collectSubSystem::reverseCollect)
                ),
                new WaitCommand(0.4),
                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::shootNote),
                        new InstantCommand(collectSubSystem::stopCollect)
                ),
                new WaitCommand(1.25),
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
