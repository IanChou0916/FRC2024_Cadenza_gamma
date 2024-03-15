package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.PositionManager;
import frc.robot.commands.position.NormalPosition;
import frc.robot.commands.position.SpeakerPosition;
import frc.robot.subsystems.ArmSubSystem;
import frc.robot.subsystems.CollectSubSystem;
import frc.robot.subsystems.ShootSubSystem;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.NORMAL;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.SPEAKER;

public class PreloadCommand extends Command {
    public PreloadCommand(
            ArmSubSystem armSubSystem,
            ShootSubSystem shootSubSystem,
            CollectSubSystem collectSubSystem){
        new SequentialCommandGroup(
                new SpeakerPosition(armSubSystem,null,NORMAL),
                new WaitCommand(0.5),
                new InstantCommand(shootSubSystem::shootNote),
                new WaitCommand(0.5),
                new InstantCommand(collectSubSystem::collectNote),
                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::stopShoot),
                        new InstantCommand(collectSubSystem::stopCollect)
                ),
                new NormalPosition(armSubSystem,null,SPEAKER)
        );

    }
}
