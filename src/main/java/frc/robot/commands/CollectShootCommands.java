package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectSubSystem;
import frc.robot.subsystems.ShootSubSystem;

import java.util.function.IntSupplier;

public class CollectShootCommands extends Command {
    private final IntSupplier collectSupplier;
    private final CollectSubSystem collectSubSystem;
    private final ShootSubSystem shootSubSystem;

    public CollectShootCommands(
            CollectSubSystem collectSubSystem,
            ShootSubSystem shootSubSystem,
            IntSupplier collectSupplier ){
        this.collectSubSystem = collectSubSystem;
        this.shootSubSystem = shootSubSystem;
        this.collectSupplier = collectSupplier;
        addRequirements(collectSubSystem);
        addRequirements(shootSubSystem);
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
                break;

            case 180:
                collectSubSystem.stopCollect();
                shootSubSystem.stopShoot();
                break;

            case 270:

                collectSubSystem.shootAMP();
                break;


        }
    }
}
