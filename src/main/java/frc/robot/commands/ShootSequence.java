package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShootConstants.SHOOT_POSITIONS;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import static frc.robot.Constants.LEDConstants.*;

public class ShootSequence extends SequentialCommandGroup{
    private final CollectSubsystem collectSubsystem;
    private final ShootSubsystem shootSubSystem;
    private final LedSubsystem ledSubSystem;
    private SHOOT_POSITIONS shootPosition;

    public ShootSequence(CollectSubsystem collectSubsystem, ShootSubsystem shootSubSystem, LedSubsystem ledSubSystem, SHOOT_POSITIONS shootPosition){
        this.collectSubsystem = collectSubsystem;
        this.shootSubSystem = shootSubSystem;
        this.ledSubSystem = ledSubSystem;
        this.shootPosition = shootPosition;
        addRequirements(collectSubsystem);
        addRequirements(shootSubSystem);
        addRequirements(ledSubSystem);

        switch (shootPosition) {
            case SPEAKER -> addCommands(
                    new ParallelCommandGroup(
                    new InstantCommand(collectSubsystem::shootAMP),
                    new InstantCommand(()-> ledSubSystem.fillRGB(SPEAKER_POSITION_COLOR)),
                    new PrintCommand("start shooting")
                ),
                new WaitCommand(0.2),
                new InstantCommand(shootSubSystem::shootNote),
                new WaitUntilCommand(shootSubSystem::shootNoteLimit),
                new InstantCommand(collectSubsystem::collectNote),
                new WaitCommand(1.0),
                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::stopShoot),
                        new InstantCommand(collectSubsystem::stopCollect),
                        new InstantCommand(()-> ledSubSystem.fillRGB(NORMAL_POSITION_COLOR)),
                        new PrintCommand("shoot Complete.")
                )
            );
            case AMP -> addCommands(
                new InstantCommand(()-> collectSubsystem.shootAMP()),
                new WaitCommand(2.0),
                new InstantCommand(collectSubsystem::stopCollect)    
            );
            case COLLECT -> addCommands(
                new ParallelCommandGroup(
                    new InstantCommand(()-> collectSubsystem.collectNote()),
                    new InstantCommand(()-> ledSubSystem.fillRGB(COLLECT_POSITION_COLOR))
                ),
                
                new WaitCommand(2.0),
                new ParallelCommandGroup(
                    new InstantCommand(collectSubsystem::stopCollect),
                    new InstantCommand(()-> ledSubSystem.fillRGB(NORMAL_POSITION_COLOR))
                )
            );
            default -> {
                new InstantCommand(()-> ledSubSystem.fillRGB(0,255,0));
                break;
            }
        }
    }

}
