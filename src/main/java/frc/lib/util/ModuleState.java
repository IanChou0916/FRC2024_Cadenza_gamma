package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleState {
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetVelocity = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetVelocity = -targetVelocity;
            targetAngle = (delta > 90) ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetVelocity, Rotation2d.fromDegrees(targetAngle));
    }
    // TODO: Need to add SDS other types
    private static double placeInAppropriate0To360Scope(double scopeReferences, double newAngle) {
        double upperBound;
        double lowerBound;
        double lowerOffset = scopeReferences % 360;
        if(lowerOffset < 0){
            upperBound = scopeReferences - lowerOffset;
            lowerBound = scopeReferences - lowerOffset - 360;
        }
        else{
            lowerBound = scopeReferences - lowerOffset;
            upperBound = scopeReferences - lowerOffset + 360;
        }
        while(newAngle > upperBound) {newAngle -= 360;}
        while(newAngle < lowerBound) {newAngle += 360;}
        if(newAngle - scopeReferences > 180)
            newAngle -= 360;
        else if(newAngle - scopeReferences < -180)
            newAngle += 360;
        return newAngle;
    }
}
