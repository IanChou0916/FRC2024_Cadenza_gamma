package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.lib.util.LimelightHelpers;
import frc.robot.subsystems.SwerveSubSystem;

public class SwerveCommands extends Command {
    private final SwerveSubSystem swerveSubSystem;
    private final DoubleSupplier translationSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;

    private final BooleanSupplier robotCentricSupplier;
    private final IntSupplier povSlowMoveSupplier;
    private final BooleanSupplier visionBooleanSup;
    private final PIDController Vision_PID = new PIDController(
            Constants.VisionConstants.VISION_PID[0],
            Constants.VisionConstants.VISION_PID[1],
            Constants.VisionConstants.VISION_PID[2]);

    public SwerveCommands(
        SwerveSubSystem swerveSubSystem,
        DoubleSupplier translationSupplier,
        DoubleSupplier strafeSupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier robotCentricSupplier,
        IntSupplier povSlowMoveSupplier,
        BooleanSupplier visionBooleanSup
    ){
        this.swerveSubSystem = swerveSubSystem;
        addRequirements(swerveSubSystem);
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
        this.povSlowMoveSupplier = povSlowMoveSupplier;
        this.visionBooleanSup = visionBooleanSup;
    }

    @Override
    public void execute(){
        double translationval;
        double strafeval;
        double rotationval = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.JoyStickConstants.DRIVE_JOYSTICK_DEADBAND);
        

        switch (povSlowMoveSupplier.getAsInt()) {
            case 0:
                translationval = Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = 0.0;
                break;
            case 45:
                translationval = Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = -Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 90:
                translationval = 0.0;
                strafeval = -Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 135:
                translationval = -Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = -Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 180:
                translationval = -Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = 0.0;
                break;
            case 225:
                translationval = -Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 270:
                translationval = 0.0;
                strafeval = Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 315:
                translationval = Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = Constants.SWERVE_POV_MOVE_SPEED;
                break;
            default:
                translationval = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.JoyStickConstants.DRIVE_JOYSTICK_DEADBAND);
                strafeval = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.JoyStickConstants.DRIVE_JOYSTICK_DEADBAND);
                break;
        }

        if (visionBooleanSup.getAsBoolean()) {
            rotationval = Vision_PID.calculate(-LimelightHelpers.getTX("limelight"), 0);

            swerveSubSystem.drive(
                new Translation2d(translationval,strafeval).times(Constants.SWERVE_MAX_SPEED),
                rotationval * Constants.SWERVE_MAX_ANGULAR_VELOCITY,
                !robotCentricSupplier.getAsBoolean()
                //false
            );
        }


        swerveSubSystem.drive(
            new Translation2d(translationval,strafeval).times(Constants.SWERVE_MAX_SPEED),
            rotationval * Constants.SWERVE_MAX_ANGULAR_VELOCITY,
            !robotCentricSupplier.getAsBoolean()
            //false
        );
    }
/*
    @Override
    public void end(boolean interrupted) {
        swerveSubSystem.stopAll();
    }

 */
}
