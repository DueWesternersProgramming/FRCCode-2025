package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ClawConstants;
import frc.robot.RobotConstants.PortConstants.CAN;

public class ClawSubsystem extends SubsystemBase {
    SparkMax clawMotor;
    SparkMaxConfig clawMotorConfig;
    static SparkClosedLoopController clawMotorController;

    public ClawSubsystem() {

        if (RobotBase.isReal()) {
            clawMotor = new SparkMax(CAN.CLAW_MOTOR, MotorType.kBrushless);
           
            clawMotorController = clawMotor.getClosedLoopController();

            clawMotorConfig = new SparkMaxConfig();

            clawMotor.configure(clawMotorConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);

            
        }
    }

    public void moveAtSpeed(double speed) {
        clawMotor.set(speed * .5);
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {

        }
    }

}
