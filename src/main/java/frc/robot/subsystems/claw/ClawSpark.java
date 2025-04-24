package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotConstants.ClawConstants;
import frc.robot.RobotConstants.PortConstants.CAN;

public class ClawSpark implements ClawSubsystemIO {

    // Define the motor controllers and other necessary components here

    SparkMax bottomSpark, topSpark;
    SparkMaxConfig bottomSparkConfig, topSparkConfig;

    public ClawSpark() {

        bottomSpark = new SparkMax(CAN.CLAW_MOTOR_1, MotorType.kBrushless);
        topSpark = new SparkMax(CAN.CLAW_MOTOR_2, MotorType.kBrushless);

        bottomSparkConfig = new SparkMaxConfig();
        topSparkConfig = new SparkMaxConfig();

        topSparkConfig.inverted(false);
        bottomSparkConfig.inverted(false);

        bottomSpark.configure(bottomSparkConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        topSpark.configure(topSparkConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void moveAtSpeed(double speed) {
        bottomSpark.set(speed);
        topSpark.set(speed);
    }

    @Override
    public double[] getCurrent() {
        return new double[] { bottomSpark.getOutputCurrent(),
                topSpark.getOutputCurrent() };
    }

    @Override
    public void intakeCoral() {
        // Implementation to intake coral
        moveAtSpeed(ClawConstants.INTAKE_CORAL_SPEED);
    }

    @Override
    public void outtakeCoral() {
        // Implementation to outtake coral
        moveAtSpeed(ClawConstants.OUTTAKE_CORAL_SPEED);
    }

    @Override
    public void intakeAlgae() {
        // Implementation to intake algae
        moveAtSpeed(ClawConstants.INTAKE_ALGAE_SPEED);
    }

    @Override
    public void outtakeAlgae() {
        // Implementation to outtake algae
        moveAtSpeed(ClawConstants.OUTTAKE_ALGAE_SPEED);
    }

    @Override
    public void updateInputs(ClawSubsystemIOInputs inputs) {
        // Update the inputs with the current state of the motors
        inputs.appliedVolts = bottomSpark.getAppliedOutput();
        inputs.currentAmps = bottomSpark.getOutputCurrent();
        inputs.temp = bottomSpark.getMotorTemperature();
    }
}