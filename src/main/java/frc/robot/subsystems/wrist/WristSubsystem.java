package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ScoringConstants.Setpoints;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotConstants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    SparkMax wristMotor;
    SparkMaxConfig wristMotorConfig;
    SparkClosedLoopController wristMotorController;
    WristSubsystemIO io;
    WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public WristSubsystem(WristSubsystemIO io) {
        this.io = io;
    }

    public Command goToCoralScoreSetpoint(Setpoints level) {
        return new InstantCommand(() -> {
            double setpoint;
            switch (level) {
                case L1:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L1
                            : WristConstants.AngleSetpoints.Coral.L1;
                    break;
                case L2:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L2
                            : WristConstants.AngleSetpoints.Coral.L2;
                    break;
                case L3:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L3
                            : WristConstants.AngleSetpoints.Coral.L3;
                    break;
                default:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.HOME
                            : WristConstants.AngleSetpoints.HOME;
                    break;
            }

            io.goToSetpoint(setpoint);

        }, this);
    }

    public Command goToCoralScoreSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            switch (level) {
                case 1:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L1
                            : WristConstants.AngleSetpoints.Coral.L1;
                    break;
                case 2:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L2
                            : WristConstants.AngleSetpoints.Coral.L2;
                    break;
                case 3:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L3
                            : WristConstants.AngleSetpoints.Coral.L3;
                    break;
                default:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.HOME
                            : WristConstants.AngleSetpoints.HOME;
                    break;
            }

            io.goToSetpoint(setpoint);

        }, this);
    }

    public Command goToAlgaeGrabSetpoint(Setpoints level) {
        return new InstantCommand(() -> {
            double setpoint;
            switch (level) {
                case L2:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L2
                            : WristConstants.AngleSetpoints.Algae.L2;
                    break;
                case L3:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L3
                            : WristConstants.AngleSetpoints.Algae.L3;
                    break;
                default:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.HOME
                            : WristConstants.AngleSetpoints.HOME;
                    break;
            }

            io.goToSetpoint(setpoint);

        }, this);
    }

    public Command goToAlgaeGrabSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            switch (level) {
                case 2:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L2
                            : WristConstants.AngleSetpoints.Algae.L2;
                    break;
                case 3:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.L3
                            : WristConstants.AngleSetpoints.Algae.L3;
                    break;
                default:
                    setpoint = CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.HOME
                            : WristConstants.AngleSetpoints.HOME;
                    break;
            }

            io.goToSetpoint(setpoint);

        }, this);
    }

    public Command goToHumanPlayerSetpoint() {
        return new InstantCommand(() -> {

            io.goToSetpoint(CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.HP
                    : WristConstants.AngleSetpoints.HP);

        }, this);
    }

    public Command goToHomeSetpoint() {
        return new InstantCommand(() -> {

            io.goToSetpoint(CowboyUtils.isSim() ? WristConstants.AngleSetpoints.SimConstants.HOME
                    : WristConstants.AngleSetpoints.HOME);

        }, this);
    }

    public void moveAtSpeed(double speed) {
        io.moveAtSpeed(speed);
    }

    public void setEncoderValue(double value) {
        io.setEncoderValue(value);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("WristSubsystem", inputs);
    }

}
