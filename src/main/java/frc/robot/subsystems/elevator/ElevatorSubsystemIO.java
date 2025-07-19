package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.RobotConstants.ScoringConstants.Setpoints;

public interface ElevatorSubsystemIO {

    @AutoLog
    public static class ElevatorSubsystemIOInputs {
        public double leftMotorTemp = 0.0;
        public double rightMotorTemp = 0.0;
        public double leftMotorPosition = 0.0;
        public double rightMotorPosition = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(ElevatorSubsystemIOInputs inputs) {

    }

    default void goToSetpoint(double position) {
    };

    /**
     * Reset the encoder value.
     * 
     * @param value The value to set the encoder to.
     */
    default void setEncoderValue(double value) {

    };

    /**
     * Set the elevator motor to a speed.
     * 
     * @param speed
     */
    default void moveAtSpeed(double speed) {
    };

}
