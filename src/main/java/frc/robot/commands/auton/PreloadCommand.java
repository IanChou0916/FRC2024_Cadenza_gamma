package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.position.NormalPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.SPEAKER;

public class PreloadCommand extends Command {
    public PreloadCommand(
            ArmSubsystem armSubSystem,
            ShootSubsystem shootSubSystem,
            CollectSubsystem collectSubSystem){
        new SequentialCommandGroup(
                new InstantCommand(() -> armSubSystem.setWristPosition(SPEAKER)),
                new WaitUntilCommand(()->Math.abs(SPEAKER.getWristPosition() - armSubSystem.getWristAngle() )< 6),
                new InstantCommand(()-> armSubSystem.setArmPosition(SPEAKER)),
                new WaitUntilCommand(()-> Math.abs(SPEAKER.getArmPosition() - armSubSystem.getArmAngle() )< 5),
                new WaitCommand(0.5),
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
