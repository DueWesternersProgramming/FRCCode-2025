package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.WristConstants;

public class WristSubsystemIOSpark implements WristSubsystemIO {
    SparkMax wristMotor;
    SparkMaxConfig wristMotorConfig;
    static SparkClosedLoopController wristMotorController;

    public WristSubsystemIOSpark() {
        wristMotor = new SparkMax(CAN.WRIST_MOTOR, MotorType.kBrushless);

        wristMotorController = wristMotor.getClosedLoopController();

        wristMotorConfig = new SparkMaxConfig();

        wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        wristMotorConfig.closedLoop.maxMotion.maxVelocity(WristConstants.MAX_MOTOR_RPM);
        wristMotorConfig.closedLoop.maxMotion.maxAcceleration(WristConstants.MAX_MOTOR_ACCELERATION);

        wristMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(.5);

        wristMotorConfig.closedLoop.pid(0.15, 0.0, 0.0);

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void goToSetpoint(double setpoint) {
        wristMotorController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void moveAtSpeed(double speed) {
        wristMotor.set(speed * .5);
    }

    @Override
    public void setEncoderValue(double value) {
        getEncoder().setPosition(value);
    }

    @Override
    public double getEncoderValue() {
        return getEncoder().getPosition();
    }

    public RelativeEncoder getEncoder() {
        return wristMotor.getEncoder();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristPosition = getEncoderValue();
        inputs.wristTemp = wristMotor.getMotorTemperature();
    }

}