package frc.robot.subsystems.wrist;

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
import frc.robot.RobotConstants.WristConstants;
import frc.robot.subsystems.ElevatorWristSim;
import frc.robot.Robot;
import com.revrobotics.spark.SparkBase.ResetMode;

public class WristSubsystem extends SubsystemBase {
    SparkMax wristMotor;
    SparkMaxConfig wristMotorConfig;
    static SparkClosedLoopController wristMotorController;

    public WristSubsystem() {

        // if (RobotBase.isReal()) {
        wristMotor = new SparkMax(CAN.WRIST_MOTOR, MotorType.kBrushless);

        wristMotorController = wristMotor.getClosedLoopController();

        wristMotorConfig = new SparkMaxConfig();

        wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        wristMotorConfig.closedLoop.maxMotion.maxVelocity(WristConstants.MAX_MOTOR_RPM);
        wristMotorConfig.closedLoop.maxMotion.maxAcceleration(WristConstants.MAX_MOTOR_ACCELERATION);
        
        wristMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(.5);

        wristMotorConfig.closedLoop.pid(0.1, 0.0, 0.0);

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // }
        // The sim combination of wrist and elevator init is done in the RobotContainer
    }

    public void goToSetpoint(double setpoint) {
        if (RobotBase.isReal()) {
            wristMotorController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
        }

    }

    public void moveAtSpeed(double speed) {
        wristMotor.set(speed * .5);
    }

    public Command goToCoralScoreSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level == 1) {
                    setpoint = WristConstants.AngleSetpoints.Coral.L1;
                } else if (level == 2) {
                    setpoint = WristConstants.AngleSetpoints.Coral.L2;
                } else if (level == 3) {
                    setpoint = WristConstants.AngleSetpoints.Coral.L3;
                } else {
                    setpoint = WristConstants.AngleSetpoints.Coral.L1;
                }
                goToSetpoint(setpoint);
            }

        }, this);
    }

    public Command goToAlgaeScoreSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level == 2) {
                    setpoint = WristConstants.AngleSetpoints.Algae.L2;
                } else if (level == 3) {
                    setpoint = WristConstants.AngleSetpoints.Algae.L3;
                } else {
                    setpoint = WristConstants.AngleSetpoints.HOME;
                }
                goToSetpoint(setpoint);
            }

        }, this);
    }

    public Command goToHumanPlayerSetpoint() {
        return new InstantCommand(() -> {
            if (RobotBase.isReal()) {
                goToSetpoint(WristConstants.AngleSetpoints.HP);
            }

        }, this);
    }


    public double getEncoderValue(){
        return wristMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()){
            SmartDashboard.putNumber("Wrist Encoder val", getEncoderValue());
            //System.out.println("EA SPORTSSS");
        }
    }

}
