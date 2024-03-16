package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangSubsystem;

import java.util.function.BooleanSupplier;
import static frc.robot.Constants.HangConstants.*;

public class HangCommands extends Command {
    private final HangSubsystem hangSubSystem;
    private final BooleanSupplier leftForwardSup;
    private final BooleanSupplier leftReverseSup;
    private final BooleanSupplier rightForwardSup;
    private final BooleanSupplier rightReverseSup;


    public HangCommands(
            HangSubsystem hangSubSystem,
            BooleanSupplier leftForwardSup, BooleanSupplier leftReverseSup,
            BooleanSupplier rightForwardSup, BooleanSupplier rightReverseSup
            ){
        this.hangSubSystem = hangSubSystem;
        this.leftForwardSup = leftForwardSup;
        this.leftReverseSup = leftReverseSup;
        this.rightForwardSup = rightForwardSup;
        this.rightReverseSup = rightReverseSup;
        addRequirements(hangSubSystem);
    }

    @Override
    public void initialize() {
        hangSubSystem.stopHangMotor();
    }
    @Override
    public void execute(){
        if(leftForwardSup.getAsBoolean()){
            hangSubSystem.setLeftHangMotor(HANG_SPEED);
        }
        if(leftReverseSup.getAsBoolean()){
            hangSubSystem.setLeftHangMotor(-HANG_SPEED);
        }
        if(rightForwardSup.getAsBoolean()){
            hangSubSystem.setRightHangMotor(HANG_SPEED);
        }
        if(rightReverseSup.getAsBoolean()){
            hangSubSystem.setRightHangMotor(-HANG_SPEED);
        }
        if(!leftForwardSup.getAsBoolean() && !rightForwardSup.getAsBoolean() && !leftReverseSup.getAsBoolean() && !rightReverseSup.getAsBoolean()){
            hangSubSystem.stopHangMotor();
        }
    }

    @Override
    public void end(boolean interrupt){
        hangSubSystem.stopHangMotor();
    }
}
