package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubSystem;

import java.util.function.BooleanSupplier;
import static frc.robot.Constants.ArmConstants.*;

public class ArmTestCommands extends Command {
    private final ArmSubSystem armSubSystem;
    private final BooleanSupplier armAddSupplier;
    private final BooleanSupplier armMinusSupplier;
    private final BooleanSupplier wristAddSupplier;
    private final BooleanSupplier wristminusSupplier;
    private double armAngle = 0;
    private double wristAngle = 0;

    private final Timer timer = new Timer();


    public ArmTestCommands(
            ArmSubSystem armSubSystem,
            BooleanSupplier armAddSupplier,
            BooleanSupplier armMinusSupplier,
            BooleanSupplier wristAddSupplier,
            BooleanSupplier wristminusSupplier){
        this.armSubSystem = armSubSystem;
        this.armAddSupplier = armAddSupplier;
        this.armMinusSupplier = armMinusSupplier;
        this.wristAddSupplier = wristAddSupplier;
        this.wristminusSupplier = wristminusSupplier;
        addRequirements(armSubSystem);
    }
    @Override
    public void initialize(){
        armAngle = armSubSystem.getArmAngle();
        wristAngle = armSubSystem.getWristAngle();
    }

    @Override
    public void execute() {
        if (armAddSupplier.getAsBoolean()) {
            if(armAngle <= ARM_FORWARD_LIMIT){armAngle+=ARM_CONTROL_VALUE;}
            timer.start();
            if (timer.get() >= ARM_CONTROL_WAITTIME && armAngle >= ARM_FORWARD_LIMIT) {
                armAngle+= ARM_CONTROL_VALUE;
                timer.restart();
            }
        }
        else if (armMinusSupplier.getAsBoolean()) {
            if(armAngle >= ARM_REVERSE_LIMIT){armAngle-=ARM_CONTROL_VALUE;}
            timer.start();
            if (timer.get() >= ARM_CONTROL_WAITTIME && armAngle <= ARM_REVERSE_LIMIT) {
                armAngle-= ARM_CONTROL_VALUE;
                timer.restart();
            }
        }
        if (wristAddSupplier.getAsBoolean()) {
            if(wristAngle <= WRIST_FORWARD_LIMIT){wristAngle+=WRIST_CONTROL_VALUE;}
            timer.start();
            if (timer.get() >= WRIST_CONTROL_WAITTIME && wristAngle >= WRIST_FORWARD_LIMIT) {
                wristAngle+= WRIST_CONTROL_VALUE;
                timer.restart();
            }
        }
        else if (wristminusSupplier.getAsBoolean()) {
            if(wristAngle >= WRIST_REVERSE_LIMIT){wristAngle-=WRIST_CONTROL_VALUE;}
            timer.start();
            if (timer.get() >= WRIST_CONTROL_WAITTIME && wristAngle <= WRIST_REVERSE_LIMIT) {
                wristAngle-= WRIST_CONTROL_VALUE;
                timer.restart();
            }
        }
        armSubSystem.setArmAngle(armAngle);
        armSubSystem.setWristAngle(wristAngle);
        SmartDashboard.putNumber("armAngle",armAngle);
        SmartDashboard.putNumber("wristAngle",wristAngle);
        SmartDashboard.putNumber("time",timer.get());
    }
}