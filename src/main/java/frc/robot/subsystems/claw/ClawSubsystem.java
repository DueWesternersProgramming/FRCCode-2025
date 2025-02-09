package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ClawConstants;
import frc.robot.RobotConstants.PortConstants.CAN;

public class ClawSubsystem extends SubsystemBase {
    SparkMax clawMotor1;
    SparkMax clawMotor2;
    SparkMaxConfig clawMotorConfig1;
    SparkMaxConfig clawMotorConfig2;


    public ClawSubsystem() {

        if (RobotBase.isReal()) {
            clawMotor1 = new SparkMax(CAN.CLAW_MOTOR_1, MotorType.kBrushless);
            clawMotor2 = new SparkMax(CAN.CLAW_MOTOR_2, MotorType.kBrushless);

            clawMotorConfig1 = new SparkMaxConfig();
            clawMotorConfig2 = new SparkMaxConfig();

            clawMotorConfig2.inverted(true);

            clawMotor1.configure(clawMotorConfig1, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
                
            clawMotor2.configure(clawMotorConfig2, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

            
        }
    }

    public void moveAtSpeed(double speed) {
        clawMotor1.set(speed);
        clawMotor2.set(speed);
    }

    public Command intakeCoral(){
        return new InstantCommand(()->{moveAtSpeed(-.5);});
    }
    public Command outtakeCoral(){
        return new InstantCommand(()->{moveAtSpeed(1);});
    }
    public Command intakeAlgae(){
        return new InstantCommand(()->{moveAtSpeed(.75);});
    }
    public Command outtakeAlgae(){
        return new InstantCommand(()->{moveAtSpeed(-1);});
    }


    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            
        }
    }

}
