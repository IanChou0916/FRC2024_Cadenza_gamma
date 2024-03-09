package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectSubSystem;
import frc.robot.subsystems.ShootSubSystem;

import java.util.function.IntSupplier;

public class CollectShootCommands extends Command {
    private IntSupplier collectSupplier;
    private CollectSubSystem collectSubSystem;
    private ShootSubSystem shootSubSystem;

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
                collectSubSystem.collectNote();
                //shootSubSystem.setSpeed(-3);
                break;
            case 90:
                collectSubSystem.stopCollect();
                shootSubSystem.stopShoot();
                break;

            case 180:
                collectSubSystem.shootAMP();
                break;
            case 270:
                shootSubSystem.setSpeed(40);
                break;
        }
    }
}
