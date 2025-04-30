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

    public default void updateInputs(ModuleIOInputs inputs) {
    }

    public default SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    public default SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public default void setDesiredState(SwerveModuleState state) {
    }

    public default void resetEncoders() {
    }
}
