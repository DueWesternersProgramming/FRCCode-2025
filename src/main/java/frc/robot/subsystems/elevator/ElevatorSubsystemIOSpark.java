package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotConstants.PortConstants.CAN;

public class ElevatorSubsystemIOSpark implements ElevatorSubsystemIO {
    SparkMax elevatorMotor1;
    SparkMax elevatorMotor2;
    SparkMaxConfig elevatorMotor1Config;
    SparkMaxConfig elevatorMotor2Config;
    SparkClosedLoopController elevatorMotor1Controller;

    public ElevatorSubsystemIOSpark() {

        elevatorMotor1 = new SparkMax(CAN.ELEVATOR_MOTOR_1, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(CAN.ELEVATOR_MOTOR_2, MotorType.kBrushless);

        elevatorMotor1Controller = elevatorMotor1.getClosedLoopController();

        elevatorMotor1Config = new SparkMaxConfig();
        elevatorMotor2Config = new SparkMaxConfig();

        elevatorMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorMotor1Config.closedLoop.maxMotion.allowedClosedLoopError(1);

        // default before auto/teleop changes it
        elevatorMotor1Config.closedLoop.maxMotion.maxVelocity(10000);
        elevatorMotor1Config.closedLoop.maxMotion.maxAcceleration(6500);

        elevatorMotor1Config.closedLoop.pid(0.5, 0.0, 2);

        elevatorMotor2Config.follow(CAN.ELEVATOR_MOTOR_1, true);

        elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        elevatorMotor1Config.softLimit
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(true).forwardSoftLimit(0).forwardSoftLimitEnabled(true);

        elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void goToSetpoint(double position) {
        elevatorMotor1Controller.setReference(position, ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0, -.2);
    }

    @Override
    public void setEncoderValue(double value) {
        elevatorMotor1.getEncoder().setPosition(value);
    }

    @Override
    public void moveAtSpeed(double speed) {
        elevatorMotor1.set(speed);
    }

    @Override
    public void updateInputs(ElevatorSubsystemIOInputs inputs) {
        inputs.leftMotorTemp = elevatorMotor1.getMotorTemperature();
        inputs.rightMotorTemp = elevatorMotor2.getMotorTemperature();
        inputs.leftMotorPosition = elevatorMotor1.getEncoder().getPosition();
        inputs.rightMotorPosition = elevatorMotor2.getEncoder().getPosition();
    }

}
