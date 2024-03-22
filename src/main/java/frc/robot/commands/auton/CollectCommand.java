package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.position.CollectPosition;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.SPEAKER;

public class CollectCommand extends Command {
    public CollectCommand(
            ArmSubsystem armSubSystem,
            ShootSubsystem shootSubSystem,
            CollectSubsystem collectSubSystem,
            XboxController operatorController) {
        new SequentialCommandGroup(
                new CollectPosition(armSubSystem, operatorController, SPEAKER),
                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::reverseNote),
                        new InstantCommand(collectSubSystem::collectNote)
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
