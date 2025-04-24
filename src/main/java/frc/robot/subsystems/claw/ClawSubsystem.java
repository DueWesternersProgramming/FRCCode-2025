package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    SparkMax clawMotor1;
    SparkMax clawMotor2;
    SparkMaxConfig clawMotorConfig1;
    SparkMaxConfig clawMotorConfig2;
    ClawSubsystemIO io;
    ClawSubsystemIOInputsAutoLogged inputs = new ClawSubsystemIOInputsAutoLogged();

    public ClawSubsystem(ClawSubsystemIO io) {
        this.io = io;
    }

    public Command setClawSpeed(double speed) {
        Logger.recordOutput("Clawsubsystem/clawspeed", speed);
        return Commands.runOnce(() -> io.moveAtSpeed(speed), this);
    }

    public Command intakeCoral() {
        return Commands.runOnce(() -> io.moveAtSpeed(ClawConstants.INTAKE_CORAL_SPEED), this);
    }

    public Command outtakeCoral() {
        return Commands.runOnce(() -> io.moveAtSpeed(ClawConstants.OUTTAKE_CORAL_SPEED), this);
    }

    public Command intakeAlgae() {
        return Commands.runOnce(() -> io.moveAtSpeed(ClawConstants.INTAKE_ALGAE_SPEED), this);
    }

    public Command outtakeAlgae() {
        return Commands.runOnce(() -> io.moveAtSpeed(ClawConstants.OUTTAKE_ALGAE_SPEED), this);
    }

    public Command stopClaw() {
        return Commands.runOnce(() -> io.moveAtSpeed(0), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ClawSubsystem", inputs);
    }

}
