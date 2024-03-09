package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubSystem;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.JoyStickConstants.*;

public class ArmCommands extends Command {

    private final ArmSubSystem armSubSystem;
    private final DoubleSupplier armSpeedSup;
    private final DoubleSupplier wristSup;

    public ArmCommands(
            ArmSubSystem armSubSystem,
            DoubleSupplier armSpeedSup,
            DoubleSupplier wristSup
    ){
        this.armSubSystem = armSubSystem;
        this.armSpeedSup = armSpeedSup;
        this.wristSup = wristSup;
        addRequirements(armSubSystem);
    }

    @Override
    public void execute(){
        armSubSystem.setArmSpeed(armSpeedSup.getAsDouble()*0.15);
        armSubSystem.setWristSpeed(wristSup.getAsDouble()*0.15);
    }
    /*
    private final BooleanSupplier armAddSupplier;
    private final BooleanSupplier armMinusSupplier;
    private final BooleanSupplier wristAddSupplier;
    private final BooleanSupplier wristminusSupplier;
    private final Timer timer = new Timer();

    private float armAngle = 0f;
    private float wristAngle = 0f;

    public ArmCommands(ArmSubSystem armSubSystem, 
    BooleanSupplier armAddSupplier,
    BooleanSupplier armMinusSupplier,
    BooleanSupplier wristAddSupplier,
    BooleanSupplier wristminusSupplier
    ){
        this.armSubSystem = armSubSystem;
        this.armAddSupplier = armAddSupplier;
        this.armMinusSupplier = armMinusSupplier;
        this.wristAddSupplier = wristAddSupplier;
        this.wristminusSupplier = wristminusSupplier;
        addRequirements(armSubSystem);
    }
    @Override
    public void initialize(){
        armAngle = (float)armSubSystem.getArmPosition();
        wristAngle = (float)armSubSystem.getWristPosition();
    }

    @Override
    public void execute(){
        if(armAddSupplier.getAsBoolean()){
            if(armAngle < ARM_FORWARD_LIMIT) {
                armAngle+=2;
            }
            timer.start();
            if(timer.get() >= ARM_CONTROL_WAITTIME && armAngle < ARM_FORWARD_LIMIT){
                armAngle+=2;
                timer.restart();
            }

        }
        else if(armMinusSupplier.getAsBoolean()) {
            if (armAngle > ARM_REVERSE_LIMIT) {
                armAngle -= 2;
            }
            timer.start();
            if (timer.get() >= ARM_CONTROL_WAITTIME && armAngle > ARM_REVERSE_LIMIT) {
                armAngle -= 2;
                timer.restart();
            }
        }
        if(wristAddSupplier.getAsBoolean()){
            if(wristAngle < WRIST_FORWARD_LIMIT) {
                wristAngle+=2;
            }
            timer.start();
            if(timer.get() >= WRIST_CONTROL_WAITTIME && wristAngle < WRIST_FORWARD_LIMIT){
                wristAngle+=2;
                timer.restart();
            }

        }
        else if(wristminusSupplier.getAsBoolean()) {
            if (wristAngle > WRIST_REVERSE_LIMIT) {
                wristAngle -= 2;
            }
            timer.start();
            if (timer.get() >=WRIST_CONTROL_WAITTIME && wristAngle > WRIST_REVERSE_LIMIT) {
                wristAngle -= 2;
                timer.restart();
            }
        }
        armSubSystem.setArmMotor(armAngle);
        armSubSystem.setWristMotor(wristAngle);

    }

     */
}
