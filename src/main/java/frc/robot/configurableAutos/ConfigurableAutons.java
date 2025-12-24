package frc.robot.configurableAutos;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.google.gson.Gson;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ConfigurableAutons {

    // StringPublisher blockTypesPublisher;
    // DoublePublisher testPublisher;
    Gson gson;

    public ConfigurableAutons() {
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // var blockTypesTopic =
        // inst.getStringTopic("DynamicAutoConfigurations/BlockTypes");
        // blockTypesTopic.setPersistent(true);
        // blockTypesPublisher = blockTypesTopic.publish();

        // var testTopic = inst.getDoubleTopic("DynamicAutoConfigurations/TestValue");
        // testTopic.setPersistent(true);
        // testPublisher = testTopic.publish();

        gson = new Gson();
        publishFullSchema();
    }

    private void publishFullSchema() {
        // may add a simpler way to define these later, but this is good for a
        // prototype.
        Map<String, Object> schema = new HashMap<>();

        schema.put("Score Coral", createBlockMap(new ArrayList<>(List.of("Reef Side", "Position", "Level"))));
        schema.put("Pickup Coral", createBlockMap(new ArrayList<>(List.of("HP station")))); // 0 for left, 1 for right

        String jsonStr = gson.toJson(schema);

        SmartDashboard.putString("DynamicAutos/BlockTypes", jsonStr);
        // blockTypesPublisher.set(jsonStr);
        // testPublisher.set(67);
    }

    private Map<String, Object> createBlockMap(ArrayList<String> params) {
        Object[] paramArray = new Object[params.size()];
        for (int i = 0; i < params.size(); i++) {
            paramArray[i] = Map.of(
                    "name", params.get(i),
                    "type", "double",
                    "default", 0.0); // all params are doubles for now, may need to change
        }

        Map<String, Object> blockMap = new HashMap<>();
        blockMap.put("params", paramArray);
        return blockMap;
    }

}
