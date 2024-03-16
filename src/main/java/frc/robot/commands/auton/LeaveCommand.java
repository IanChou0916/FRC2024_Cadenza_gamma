package frc.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class LeaveCommand extends Command {
    public LeaveCommand(SwerveSubsystem swerveSubSystem){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,1.0,0);
        new SequentialCommandGroup(
                new InstantCommand(swerveSubSystem::resetModulesToAbsolute),
                //new InstantCommand(swerveSubSystem.driveRobotRelative(chassisSpeeds)),
                new WaitCommand(1.0),
                new InstantCommand(swerveSubSystem::stopAll));
    }
}
