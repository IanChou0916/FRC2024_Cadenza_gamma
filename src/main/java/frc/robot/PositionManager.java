package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.position.AmpPosition;
import frc.robot.commands.position.CollectPosition;
import frc.robot.commands.position.SpeakerPosition;
import frc.robot.subsystems.ArmSubSystem;
import frc.robot.subsystems.CollectSubSystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ARM_POSITIONS.*;

public class PositionManager {
    private final ArmSubSystem armSubSystem;
    private final CollectSubSystem collectSubSystem;
    private final XboxController operatorController;
    private ARM_POSITIONS presentPosition;

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

}
