package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public double turningTemp = 0.0;
        public double driveTemp = 0.0;
        // public SwerveModuleState moduleState = new SwerveModuleState();
        // public SwerveModulePosition modulePosition = new SwerveModulePosition();
    }

    default void updateInputs(ModuleIOInputs inputs) {
    }

    default SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    default SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    default void setDesiredState(SwerveModuleState state) {
    }

    default void resetEncoders() {
    }
}
