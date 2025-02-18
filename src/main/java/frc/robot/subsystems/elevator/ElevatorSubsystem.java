package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
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

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevatorMotor1;
    SparkMax elevatorMotor2;
    SparkMaxConfig elevatorMotor1Config;
    SparkMaxConfig elevatorMotor2Config;
    ProfiledPIDController pidController;

    public ElevatorSubsystem() {

        elevatorMotor1 = new SparkMax(CAN.ELEVATOR_MOTOR_1, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(CAN.ELEVATOR_MOTOR_2, MotorType.kBrushless);

        elevatorMotor1Config = new SparkMaxConfig();
        elevatorMotor2Config = new SparkMaxConfig();

        elevatorMotor2Config.follow(CAN.ELEVATOR_MOTOR_1, true);

        elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pidController = new ProfiledPIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, ElevatorConstants.CONSTRAINTS);

        new ElevatorWristSim();
    }

    public void goToSetpoint(double setpoint) {

        if (RobotBase.isReal()) {
            double output = pidController.calculate(getEncoder().getPosition(), setpoint);
            elevatorMotor1.set(output + ElevatorConstants.FEEDFORWARD);
        }
    }
    public void setEncoderValue(double value) {
        // In rotations
        elevatorMotor1.getEncoder().setPosition(value);
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
                //ElevatorWristSim.goToScoreSetpoint(level);// Passes in the L1-L3 into the sim logic, this needs some work.
            }
        }, this);
    }

    public Command goToHumanPlayerPickup() {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                goToSetpoint(ElevatorConstants.HeightSetpoints.HP);
            } else {
                ElevatorWristSim.setElevatorToHeight(ElevatorConstants.SimConstants.HP);// Passes in the L1-L3 into the sim logic
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
