package frc.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.subsystems.ElevatorWristSim;
import frc.robot.Robot;
import frc.robot.RobotConstants.ElevatorConstants;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevatorMotor1;
    SparkMax elevatorMotor2;
    SparkMaxConfig elevatorMotor1Config;
    SparkMaxConfig elevatorMotor2Config;
    static SparkClosedLoopController elevatorMotor1Controller;

    public ElevatorSubsystem() {

        if (RobotBase.isReal()) {
            elevatorMotor1 = new SparkMax(CAN.ELEVATOR_MOTOR_1, MotorType.kBrushless);
            elevatorMotor2 = new SparkMax(CAN.ELEVATOR_MOTOR_2, MotorType.kBrushless);

            elevatorMotor1Controller = elevatorMotor1.getClosedLoopController();

            elevatorMotor1Config = new SparkMaxConfig();
            elevatorMotor2Config = new SparkMaxConfig();

            elevatorMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            elevatorMotor1Config.closedLoop.maxMotion.maxVelocity(ElevatorConstants.MAX_MOTOR_RPM);
            elevatorMotor1Config.closedLoop.maxMotion.maxAcceleration(ElevatorConstants.MAX_MOTOR_ACCELERATION);

            elevatorMotor1Config.closedLoop.pid(0.1, 0.0, 0.0);

            elevatorMotor2Config.follow(CAN.ELEVATOR_MOTOR_1);
            elevatorMotor2Config.inverted(true);

            elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);

            elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        } else {
            new ElevatorWristSim();
        }

    }

    public static void goToSetpoint(double setpoint) {
        // Add code here to move the elevator to the scoring height
        if (RobotBase.isReal()) {
            elevatorMotor1Controller.setReference(setpoint, ControlType.kMAXMotionPositionControl);
        }

    }

    public static Command goToScoreSetpoint(Supplier<Integer> level, ElevatorSubsystem elevatorSubsystem) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level.get() == 1) {
                    setpoint = ElevatorConstants.ScoringHeight.L1;
                } else if (level.get() == 2) {
                    setpoint = ElevatorConstants.ScoringHeight.L2;
                } else if (level.get() == 3) {
                    setpoint = ElevatorConstants.ScoringHeight.L3;
                } else {
                    setpoint = ElevatorConstants.ScoringHeight.L1;
                }
                goToSetpoint(setpoint);
            } else {
                ElevatorWristSim.setElevatorLength(level.get());
            }
            System.out.println("Going to level: " + level.get());
        }, elevatorSubsystem);

    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {

        } else {
            ElevatorWristSim.update();
        }
    }

}
