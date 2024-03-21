package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.util.function.IntSupplier;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.AMP;

public class CollectShooTestCommands extends Command {
    private final IntSupplier collectSupplier;
    private final CollectSubsystem collectSubSystem;
    private final ArmSubsystem armSubsystem;
    private final ShootSubsystem shootSubSystem;


    public CollectShooTestCommands(
            CollectSubsystem collectSubSystem,
            ShootSubsystem shootSubSystem,
            ArmSubsystem armSubsystem,
            IntSupplier collectSupplier ){
        this.collectSubSystem = collectSubSystem;
        this.shootSubSystem = shootSubSystem;
        this.armSubsystem = armSubsystem;
        this.collectSupplier = collectSupplier;

        addRequirements(collectSubSystem);
        addRequirements(shootSubSystem);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        switch(collectSupplier.getAsInt()){
            case 0:
                shootSubSystem.shootNote();
                break;

            case 90:
                // TODO : added LED subSystem to notify drivers.
                collectSubSystem.collectNote();
                //shootSubSystem.reverseNote();
                break;

            case 180:

                collectSubSystem.stopCollect();
                shootSubSystem.stopShoot();
                break;

            case 270:
                if(armSubsystem.getPosition() == AMP) {
                    collectSubSystem.shootAMP();
                    shootSubSystem.reverseNote();
                }
                else {
                    collectSubSystem.reverseCollect();
                }
                break;


        }
    }
}
