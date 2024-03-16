
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.AMP;

public class ArmCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final BooleanSupplier armAddSupplier;
    private final BooleanSupplier armMinusSupplier;
    private final BooleanSupplier wristAddSupplier;
    private final BooleanSupplier wristmiunsSupplier;
    private double armAngle = 0;
    private double wristAngle = 0;

    private final Timer timer = new Timer();


    public ArmCommand(
            ArmSubsystem armSubsystem,
            BooleanSupplier armAddSupplier,
            BooleanSupplier armMinusSupplier,
            BooleanSupplier wristAddSupplier,
            BooleanSupplier wristmiunsSupplier){
        this.armSubsystem = armSubsystem;
        this.armAddSupplier = armAddSupplier;
        this.armMinusSupplier = armMinusSupplier;
        this.wristAddSupplier = wristAddSupplier;
        this.wristmiunsSupplier = wristmiunsSupplier;
        addRequirements(armSubsystem);
    }
    @Override
    public void initialize(){
        armAngle = armSubsystem.getArmAngle();
        wristAngle = armSubsystem.getWristAngle();
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
            if(armAngle >= ARM_REVERSE_LIMIT && (armAngle >= 15 || wristAngle>15)){armAngle-=ARM_CONTROL_VALUE;}
            timer.start();
            if (timer.get() >= ARM_CONTROL_WAITTIME && armAngle <= ARM_REVERSE_LIMIT ) {
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
        else if (wristmiunsSupplier.getAsBoolean()) {
            if(wristAngle >= WRIST_REVERSE_LIMIT){wristAngle-=WRIST_CONTROL_VALUE;}
            timer.start();
            if (timer.get() >= WRIST_CONTROL_WAITTIME && wristAngle <= WRIST_REVERSE_LIMIT) {
                wristAngle-= WRIST_CONTROL_VALUE;
                timer.restart();
            }
        }
        armSubsystem.setArmPosition(armAngle);
        armSubsystem.setWristPosition(wristAngle);
        SmartDashboard.putNumber("armAngle",armAngle);
        SmartDashboard.putNumber("wristAngle",wristAngle);

        //SmartDashboard.putNumber("time",timer.get());
    }
    private void setArmAngle(double angle){
        armAngle = angle;
        updateCommand();
    }
    private void setWristAngle(double angle){
        wristAngle = angle;
        updateCommand();
    }
    private void updateCommand(){
        armSubsystem.setDefaultCommand(this);
    }


}
