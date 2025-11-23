package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double gyroAngle = 0.0;
    }

    default double getGyroYawAngle() {
        return 0.0;
    }

    default double getGyroPitchAngle() {
        return 0.0;
    }

    default double getGyroRollAngle() {
        return 0.0;
    }

    default Rotation2d getGyroYawRotation2d() {
        return Rotation2d.fromDegrees(0.0);
    }

    default void reset() {
        // default implementation does nothing ;) - Harrison learning AKit
    }

    default void setGyroAngle(double angle) {
        // default implementation does nothing ;) - Harrison learning AKit
    }

    default double getVelocityX() {
        return 0.0;
    }

    default double getVelocityY() {
        return 0.0;
    }

    default double getRate() {
        return 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;
        inputs.gyroAngle = 0.0;
    }
}