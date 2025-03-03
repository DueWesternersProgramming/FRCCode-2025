package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.subsystems.ElevatorWristSim;
import frc.robot.RobotConstants.ElevatorConstants;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

//@Logged
public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevatorMotor1;
    SparkMax elevatorMotor2;
    SparkMaxConfig elevatorMotor1Config;
    SparkMaxConfig elevatorMotor2Config;
    static SparkClosedLoopController elevatorMotor1Controller;

    public ElevatorSubsystem() {

        elevatorMotor1 = new SparkMax(CAN.ELEVATOR_MOTOR_1, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(CAN.ELEVATOR_MOTOR_2, MotorType.kBrushless);

        elevatorMotor1Controller = elevatorMotor1.getClosedLoopController();

        elevatorMotor1Config = new SparkMaxConfig();
        elevatorMotor2Config = new SparkMaxConfig();

        elevatorMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorMotor1Config.closedLoop.maxMotion.allowedClosedLoopError(.75);
        elevatorMotor1Config.closedLoop.maxMotion.maxVelocity(ElevatorConstants.MAX_MOTOR_RPM);
        elevatorMotor1Config.closedLoop.maxMotion.maxAcceleration(ElevatorConstants.MAX_MOTOR_ACCELERATION);

        elevatorMotor1Config.closedLoop.pid(0.1, 0.0, 0.5);

        elevatorMotor2Config.follow(CAN.ELEVATOR_MOTOR_1, true);

        elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        elevatorMotor1Config.softLimit
                .reverseSoftLimit(-71)
                .reverseSoftLimitEnabled(true).forwardSoftLimit(0).forwardSoftLimitEnabled(true);

        elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        // } else {
        new ElevatorWristSim();
    }

    public void setSoftLimitEnabled(boolean isEnabled) {
        elevatorMotor1Config.softLimit.forwardSoftLimitEnabled(isEnabled).reverseSoftLimitEnabled(isEnabled);
    }

    public void goToSetpoint(double setpoint) {
        // Add code here to move the elevator to the scoring height
        if (RobotBase.isReal()) {
            elevatorMotor1Controller.setReference(setpoint, ControlType.kMAXMotionPositionControl,
                    ClosedLoopSlot.kSlot0, -.2);
        }
    }

    public void setEncoderValue(double value) {
        // In rotations
        elevatorMotor1.getEncoder().setPosition(value);
    }

    public void setMotorVoltage(double volts) {
        elevatorMotor1.setVoltage(volts);
    }

    public Command goToCoralScoreSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level == 1) {
                    setpoint = ElevatorConstants.HeightSetpoints.Coral.L1;
                } else if (level == 2) {
                    setpoint = ElevatorConstants.HeightSetpoints.Coral.L2;
                } else if (level == 3) {
                    setpoint = ElevatorConstants.HeightSetpoints.Coral.L3;
                } else {
                    setpoint = ElevatorConstants.HeightSetpoints.HOME;
                }
                goToSetpoint(setpoint);
            } else {
                ElevatorWristSim.goToScoreSetpoint(level);// Passes in the L1-L3 into the sim logic
            }
        }, this);
    }

    public Command goToAlgaeGrabSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level == 2) {
                    setpoint = ElevatorConstants.HeightSetpoints.Algae.L2;
                } else if (level == 3) {
                    setpoint = ElevatorConstants.HeightSetpoints.Algae.L3;
                } else {
                    setpoint = ElevatorConstants.HeightSetpoints.HOME;
                }
                goToSetpoint(setpoint);
            } else {
                // ElevatorWristSim.goToScoreSetpoint(level);// Passes in the L1-L3 into the sim
                // logic, this needs some work.
            }
        }, this);
    }

    public Command goToHumanPlayerPickup() {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                goToSetpoint(ElevatorConstants.HeightSetpoints.HP);
            } else {
                ElevatorWristSim.goToHumanPlayerSetpoint();// Passes in the L1-L3 into the
                                                           // sim logic
            }
        }, this);
    }

    public void moveAtSpeed(double speed) {
        elevatorMotor1.set(speed * .5);
    }

    // public Command homeElevator() {
    // return this.run(() -> elevatorMotor1.setVoltage(1)).until(() ->
    // getCurrentDraw() > 30.0)
    // .finallyDo(() -> setEncoderValue(0));
    // }

    public double getCurrentDraw() {
        return elevatorMotor1.getOutputCurrent();
    }

    public RelativeEncoder getEncoder() {
        return elevatorMotor1.getEncoder();
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("elevator encoder pos", elevatorMotor1.getEncoder().getPosition());
            // SmartDashboard.putNumber("elevator motor current draw", getCurrentDraw());
        } else {
        }
    }

}
