package frc.robot.subsystems.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNavIO io;
    QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

    private final QuestCalibration calibration = new QuestCalibration();

    public QuestNavSubsystem(QuestNavIO io) {
        this.io = io;
    }

    public void setRobotPose(Pose2d pose) {
        io.setRobotPose(pose);
    }

    public Pose2d getRobotPose() {
        return io.getRobotPose();
    }

    public Boolean isConnected() {
        return io.isConnected();
    }

    public Command calibrateCommand(DriveSubsystem driveSubsystem) {
        return calibration.determineOffsetToRobotCenter(
                driveSubsystem,
                io.getRobotPose());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("QuestNavSubsystem", inputs);

        // Do filtering here in the future...
        Boolean isPoseWithinTolerance = true;

        if (DriverStation.isEnabled() && RobotState.isQuestNavPoseReset && isPoseWithinTolerance) {
            RobotState.offerQuestMeasurement(new TimestampedPose(getRobotPose(),
                    inputs.timestamp));
        }
    }

}
