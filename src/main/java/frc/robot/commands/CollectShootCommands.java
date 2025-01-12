
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static frc.robot.Constants.LEDConstants.*;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.AMP;

public class CollectShootCommands extends Command {

    private final CollectSubsystem collectSubSystem;
    private final ShootSubsystem shootSubSystem;
    private final LedSubsystem ledSubSystem;
    //private final XboxController operatorController;
    private final IntSupplier collectSupplier;
    private final DoubleSupplier reverseSupplier;
    private final Timer timer = new Timer();


    public CollectShootCommands(
            CollectSubsystem collectSubSystem,
            ShootSubsystem shootSubSystem,
            LedSubsystem ledSubSystem,
            IntSupplier collectSupplier,
            DoubleSupplier reverseSupplier
    ){
        this.collectSubSystem = collectSubSystem;
        this.shootSubSystem = shootSubSystem;
        //this.armSubsystem = armSubsystem;
        this.ledSubSystem = ledSubSystem;
        this.collectSupplier = collectSupplier;
        this.reverseSupplier = reverseSupplier;

        addRequirements(collectSubSystem);
        addRequirements(shootSubSystem);
        //addRequirements(armSubsystem);
        addRequirements(ledSubSystem);
    }

    @Override
    public void initialize(){
        timer.reset();
        collectSubSystem.stopCollect();
        shootSubSystem.stopShoot();

    }
    @Override
    public void execute(){
        if(reverseSupplier.getAsDouble()>0.2){
            shootSubSystem.reverseNote();
            collectSubSystem.shootAMP();
        }
        switch(collectSupplier.getAsInt()){
            case 0 :
                
                shootNoteSequence();

                break;

            case 90:
                collectNoteSequence();
                // TODO : added LED subSystem to notify drivers.
                /* 
                collectSubSystem.collectNote();
                if(!collectSubSystem.getCollectLimit()){
                    //operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0.5);
                    ledSubSystem.runOnce(()->{
                        ledSubSystem.fillRGB(GET_NOTE_COLOR);
                        //new WaitCommand(2.0);
                        //ledSubSystem.fillRGB(NORMAL_POSITION_COLOR);
                    });

                }
                else {
                    ledSubSystem.fillRGB(NORMAL_POSITION_COLOR);
                    ledSubSystem.write();
                    //operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0);
                }

                //shootSubSystem.reverseNote();
                break;
                */

            case 180:

                collectSubSystem.stopCollect();
                shootSubSystem.stopShoot();
                break;

            case 225:
                collectSubSystem.shootAMP();
                shootSubSystem.reverseNote();
                break;
            case 270:
                if(collectSubSystem.getTuningPosition() == AMP){
                    collectSubSystem.shootAMP();
                    shootSubSystem.reverseNote();
                }
                else {
                    collectSubSystem.reverseCollect();
                    shootSubSystem.reverseNote();
                }

                break;
            default:
                if(!collectSubSystem.getCollectLimit()){
                    //operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0.5);
                    ledSubSystem.runOnce(()->{
                        ledSubSystem.fillRGB(GET_NOTE_COLOR);
                        new WaitCommand(2.0);
                        ledSubSystem.fillRGB(NORMAL_POSITION_COLOR);
                    });

                }
                else {
                    ledSubSystem.fillRGB(NORMAL_POSITION_COLOR);
                    ledSubSystem.write();
                    //operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0);
                }
                
        }
    }
    private SequentialCommandGroup ledLight(){
        return new SequentialCommandGroup(
                new InstantCommand(()->ledSubSystem.fillRGB(GET_NOTE_COLOR)),
                new WaitCommand(2.0),
                new InstantCommand(()->ledSubSystem.fillRGB(NORMAL_POSITION_COLOR))

        );
    }

    public SequentialCommandGroup collectNoteSequence() {
        return new SequentialCommandGroup(
                new InstantCommand(()->collectSubSystem.collectNote()),
                new WaitCommand(2.0),
                new InstantCommand(()->collectSubSystem.stopCollect())
        );
    }
    public SequentialCommandGroup shootNoteSequence() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new InstantCommand(shootSubSystem::reverseNote),
                    new InstantCommand(()-> ledSubSystem.fillRGB(SPEAKER_POSITION_COLOR)),
                    new PrintCommand("start shooting")
                ),
                new WaitCommand(0.1),
                new InstantCommand(shootSubSystem::shootNote),
                new WaitUntilCommand(shootSubSystem::shootNoteLimit),
                new InstantCommand(collectSubSystem::collectNote),
                new WaitCommand(1.0),
                new ParallelCommandGroup(
                        new InstantCommand(shootSubSystem::stopShoot),
                        new InstantCommand(collectSubSystem::stopCollect),
                        new InstantCommand(()-> ledSubSystem.fillRGB(NORMAL_POSITION_COLOR)),
                        new PrintCommand("shoot Complete.")
                )
        );
    }
}
