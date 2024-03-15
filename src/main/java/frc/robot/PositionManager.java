package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.position.AmpPosition;
import frc.robot.commands.position.CollectPosition;
import frc.robot.commands.position.NormalPosition;
import frc.robot.commands.position.SpeakerPosition;
import frc.robot.subsystems.ArmSubSystem;
import frc.robot.subsystems.CollectSubSystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.*;

public class PositionManager {
    protected final ArmSubSystem armSubSystem;
    protected final CollectSubSystem collectSubSystem;
    protected final XboxController operatorController;
    protected ARM_POSITIONS presentPosition;

    public PositionManager(
            ArmSubSystem armSubSystem,
            CollectSubSystem collectSubSystem,
            XboxController operatorController,
            ARM_POSITIONS presentPosition){
        this.armSubSystem = armSubSystem;
        this.collectSubSystem = collectSubSystem;
        this.operatorController = operatorController;
        this.presentPosition = presentPosition;
    }

    public void setPresentPosition(ARM_POSITIONS Position){
        if(presentPosition ==  AMP) System.out.println("AMP");
        else if(presentPosition ==  COLLECT) System.out.println("COLLECT");
        else if(presentPosition == NORMAL) System.out.println("NORMAL");
        else if(presentPosition == SPEAKER) System.out.println("SPEAKER");
        presentPosition = Position;
    }


    public SequentialCommandGroup TargetAmpPosition(){
        AmpPosition ampPosition = new AmpPosition(armSubSystem,operatorController,presentPosition);
        setPresentPosition(AMP);
        return ampPosition;
    }
    public SequentialCommandGroup TargetCollectPosition(){
        CollectPosition collectPosition = new CollectPosition(armSubSystem,operatorController,presentPosition);
        setPresentPosition(COLLECT);
        return collectPosition;
    }
    public SequentialCommandGroup TargetSpeakerPosition(){
        SpeakerPosition speakerPosition = new SpeakerPosition(armSubSystem,operatorController,presentPosition);
        setPresentPosition(SPEAKER);
        return speakerPosition;
    }
    public SequentialCommandGroup TargetNormalPosition(){
        NormalPosition normalPosition = new NormalPosition(armSubSystem,operatorController,presentPosition);
        setPresentPosition(NORMAL);
        return normalPosition;
    }



}
