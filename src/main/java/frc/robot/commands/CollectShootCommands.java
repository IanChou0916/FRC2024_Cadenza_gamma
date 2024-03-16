
package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static frc.robot.Constants.LEDConstants.*;

import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.AMP;

public class CollectShootCommands extends Command {

    private final CollectSubsystem collectSubSystem;
    //private final ArmSubsystem armSubsystem;
    private final ShootSubsystem shootSubSystem;
    private final LedSubsystem ledSubSystem;
    //private final XboxController operatorController;
    private final IntSupplier collectSupplier;
    private final Timer timer = new Timer();


    public CollectShootCommands(
            CollectSubsystem collectSubSystem,
            ShootSubsystem shootSubSystem,
            //ArmSubsystem armSubsystem,
            LedSubsystem ledSubSystem,
            IntSupplier collectSupplier
    ){
        this.collectSubSystem = collectSubSystem;
        this.shootSubSystem = shootSubSystem;
        //this.armSubsystem = armSubsystem;
        this.ledSubSystem = ledSubSystem;
        this.collectSupplier = collectSupplier;

        addRequirements(collectSubSystem);
        addRequirements(shootSubSystem);
        //addRequirements(armSubsystem);
        addRequirements(ledSubSystem);
    }

    @Override
    public void initialize(){
        timer.reset();

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
                if(!collectSubSystem.getCollectLimit()){
                    //operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0.5);
                    ledSubSystem.fillRGB(GET_NOTE_COLOR);
                    ledSubSystem.write();
                }
                else {

                    //operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0);
                }

                //shootSubSystem.reverseNote();
                break;

            case 180:

                collectSubSystem.stopCollect();
                shootSubSystem.stopShoot();
                break;

            case 270:
                if(collectSubSystem.getTuningPosition() == AMP){
                    collectSubSystem.shootAMP();
                }
                else {
                    collectSubSystem.reverseCollect();
                }

                break;
            default:
                if(!collectSubSystem.getCollectLimit()){
                    //operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0.5);
                    ledSubSystem.fillRGB(GET_NOTE_COLOR);
                    ledSubSystem.write();
                }
                else {
                    ledSubSystem.fillRGB(NORMAL_POSITION_COLOR);
                    ledSubSystem.write();
                    //operatorController.setRumble(GenericHID.RumbleType.kRightRumble,0);
                }
        }

    }
}
