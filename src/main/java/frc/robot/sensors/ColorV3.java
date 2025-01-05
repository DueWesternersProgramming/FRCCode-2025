package frc.robot.sensors;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

public class ColorV3 {

	private ColorSensorV3 colorSensor;

    public ColorV3(Port port) {
		this.colorSensor = new ColorSensorV3(port);
	}

    public Color getColor(){
        return colorSensor.getColor();
    }
}
