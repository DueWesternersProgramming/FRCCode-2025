package frc.robot.configurableAutos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicAutoRegistry {
    Map<String, AutoCommandDef> registry;
    Gson gson;

    public DynamicAutoRegistry() {
        registry = new HashMap<>();
        gson = new Gson();
    }

    public void registerCommand(AutoCommandDef commandDef) {
        registry.put(commandDef.name(), commandDef);

    }

    public AutoCommandDef getCommandDef(String name) {
        return registry.get(name);
    }

    public Command buildCommand(String name, Map<String, Integer> params) {
        AutoCommandDef def = registry.get(name);
        if (def != null) {
            return def.factory().create(params);
        }
        return null;
    }

    public void publishCommands() {
        Map<String, Object> schema = new HashMap<>();

        for (String name : registry.keySet()) {
            AutoCommandDef def = registry.get(name);

            List<Map<String, Object>> paramList = new ArrayList<>();

            for (AutoParamDef param : def.params()) {
                Map<String, Object> paramEntry = new HashMap<>();
                paramEntry.put("name", param.name());
                paramEntry.put("default", param.defaultValue());

                paramList.add(paramEntry);
            }

            schema.put(name, Map.of("params", paramList));
        }

        String jsonStr = gson.toJson(schema);
        SmartDashboard.putString("DynamicAutos/BlockTypes", jsonStr);
    }

}
