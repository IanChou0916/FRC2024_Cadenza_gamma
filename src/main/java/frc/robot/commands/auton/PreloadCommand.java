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

public class PreloadCommand extends Command {
    public PreloadCommand(
            ArmSubsystem armSubSystem,
            ShootSubsystem shootSubSystem,
            CollectSubsystem collectSubSystem,
            XboxController operatorController){
        new SequentialCommandGroup(
                new SpeakerPosition(armSubSystem,operatorController,NORMAL),
                new InstantCommand(shootSubSystem::shootNote),
                new WaitCommand(0.5),
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
