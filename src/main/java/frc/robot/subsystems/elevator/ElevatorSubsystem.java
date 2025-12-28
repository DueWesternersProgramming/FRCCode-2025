package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ScoringConstants.Setpoints;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.utils.CowboyUtils;

public class ElevatorSubsystem extends SubsystemBase {
    ElevatorSubsystemIO io;
    ElevatorSubsystemIOInputsAutoLogged inputs = new ElevatorSubsystemIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorSubsystemIO io) {
        this.io = io;
    }

    public void setEncoderValue(double value) {
        io.setEncoderValue(value);
    }

    public Command goToCoralScoreSetpoint(Setpoints level) {
        return new InstantCommand(() -> {
            double setpoint;
            switch (level) {
                case L1:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L1
                            : ElevatorConstants.HeightSetpoints.Coral.L1;
                    break;
                case L2:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L2
                            : ElevatorConstants.HeightSetpoints.Coral.L2;
                    break;
                case L3:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L3
                            : ElevatorConstants.HeightSetpoints.Coral.L3;
                    break;
                default:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.HOME
                            : ElevatorConstants.HeightSetpoints.HOME;
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
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L1
                            : ElevatorConstants.HeightSetpoints.Coral.L1;
                    break;
                case 2:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L2
                            : ElevatorConstants.HeightSetpoints.Coral.L2;
                    break;
                case 3:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L3
                            : ElevatorConstants.HeightSetpoints.Coral.L3;
                    break;
                default:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.HOME
                            : ElevatorConstants.HeightSetpoints.HOME;
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
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L2
                            : ElevatorConstants.HeightSetpoints.Algae.L2;
                    break;
                case L3:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L3
                            : ElevatorConstants.HeightSetpoints.Algae.L3;
                    break;
                default:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.HOME
                            : ElevatorConstants.HeightSetpoints.HOME;
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
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L2
                            : ElevatorConstants.HeightSetpoints.Algae.L2;
                    break;
                case 3:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.L3
                            : ElevatorConstants.HeightSetpoints.Algae.L3;
                    break;
                default:
                    setpoint = CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.HOME
                            : ElevatorConstants.HeightSetpoints.HOME;
                    break;
            }

            io.goToSetpoint(setpoint);

        }, this);
    }

    public Command goToHumanPlayerPickup() {
        return new InstantCommand(() -> {
            io.goToSetpoint(CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.HP
                    : ElevatorConstants.HeightSetpoints.HP);
        }, this);
    }

    public Command goToHomeSetpoint() {
        return new InstantCommand(() -> {
            io.goToSetpoint(CowboyUtils.isSim() ? ElevatorConstants.HeightSetpoints.SimConstants.HOME
                    : ElevatorConstants.HeightSetpoints.HOME);
        }, this);
    }

    public void moveAtSpeed(double speed) {
        io.moveAtSpeed(speed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorSubsystem", inputs);
    }

}
