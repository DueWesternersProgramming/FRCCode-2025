package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristSubsystemIO {
    @AutoLog
    public static class WristIOInputs {
        public double wristPosition = 0.0;
        public double wristTemp = 0.0;
    }

    default void goToSetpoint(double setpoint) {
        // Default implementation does nothing
    }

    default void moveAtSpeed(double speed) {
        // Default implementation does nothing
    }

    default void setEncoderValue(double value) {
        // Default implementation does nothing
    }

    default double getEncoderValue() {
        return 0.0; // Default implementation returns 0
    }

    default void updateInputs(WristIOInputs inputs) {
        inputs.wristPosition = 0;
        inputs.wristTemp = 0;
    }

}
