
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.position.AmpPosition;
import frc.robot.commands.position.CollectPosition;
import frc.robot.commands.position.NormalPosition;
import frc.robot.commands.position.SpeakerPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.*;

public class PositionManager  {
    protected final ArmSubsystem armSubSystem;
    protected final CollectSubsystem collectSubSystem;
    protected final XboxController operatorController;
    protected ARM_POSITIONS presentPosition;

    public PositionManager(
            ArmSubsystem armSubSystem,
            CollectSubsystem collectSubSystem,
            XboxController operatorController,
            ARM_POSITIONS presentPosition){
        this.armSubSystem = armSubSystem;
        this.collectSubSystem = collectSubSystem;
        this.operatorController = operatorController;
        this.presentPosition = presentPosition;
    }

    public void setPresentPosition(ARM_POSITIONS Position){
        /**
         * @param Position
         */
        presentPosition = Position;
        collectSubSystem.setTuningPosition(Position); // for AMP use;
        //armSubSystem.setPosition(presentPosition);
    }
    public ARM_POSITIONS getPresentPosition(){
        return presentPosition;
    }


    public SequentialCommandGroup TargetAmpPosition(){
        AmpPosition ampPosition = new AmpPosition(armSubSystem,operatorController,presentPosition);
        setPresentPosition(AMP);
        collectSubSystem.setTuningPosition(AMP);
        return ampPosition;
    }
    public SequentialCommandGroup TargetCollectPosition(){
        CollectPosition collectPosition = new CollectPosition(armSubSystem,operatorController,presentPosition);
        setPresentPosition(COLLECT);
        collectSubSystem.setTuningPosition(COLLECT);
        return collectPosition;
    }
    public SequentialCommandGroup TargetSpeakerPosition(){
        SpeakerPosition speakerPosition = new SpeakerPosition(armSubSystem,operatorController,presentPosition);
        setPresentPosition(SPEAKER);
        collectSubSystem.setTuningPosition(SPEAKER);
        return speakerPosition;
    }
    public SequentialCommandGroup TargetNormalPosition(){
        NormalPosition normalPosition = new NormalPosition(armSubSystem,operatorController,presentPosition);
        setPresentPosition(NORMAL);
        collectSubSystem.setTuningPosition(NORMAL);
        return normalPosition;
    }



}
