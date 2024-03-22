package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.position.CollectPosition;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.SPEAKER;

public class CollectCommand extends SequentialCommandGroup {
    public CollectCommand(ShootSubsystem shootSubSystem, CollectSubsystem collectSubSystem) {
        this.addRequirements(shootSubSystem);
        this.addRequirements(collectSubSystem);
        addCommands(

                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::reverseNote),
                        new InstantCommand(collectSubSystem::collectNote)
                ),
                new WaitCommand(2.0),
                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::stopShoot),
                        new InstantCommand(collectSubSystem::stopCollect)
                )
                //new NormalPosition(armSubSystem,null,SPEAKER)
        );
    }
}
