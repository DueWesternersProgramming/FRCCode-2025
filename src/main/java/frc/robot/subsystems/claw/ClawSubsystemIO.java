package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawSubsystemIO {

    @AutoLog
    public static class ClawSubsystemIOInputs {
        public double temp = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    // Set as defaults for log replay and nothing is created due to no hardware
    // needed.

    /** Update the set of loggable inputs. */
    public default void updateInputs(ClawSubsystemIOInputs inputs) {
        // default implementation does nothing ;) - Harrison learning AKit
    }

    default void moveAtSpeed(double speed) {
    };

    default double[] getCurrent() {
        return new double[] { 0.0, 0.0 };
    };

    default void intakeCoral() {
    };

    default void outtakeCoral() {
    };

    default void intakeAlgae() {
    };

    default void outtakeAlgae() {
    };

}
