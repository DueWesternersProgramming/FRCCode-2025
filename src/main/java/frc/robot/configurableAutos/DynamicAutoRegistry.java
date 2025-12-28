package frc.robot.configurableAutos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
            System.out.println("Building command: " + name + " with params: " + params);
            return def.factory().create(params);
        }
        return null;
    }

    public Command buildAuto() {
        System.out.println("Building auto...");

        SequentialCommandGroup cmd = new SequentialCommandGroup(
                new InstantCommand(() -> System.out.println("EA Sports"))
                        .withName("Startup Print"));

        String routineJson = SmartDashboard.getString(
                "DynamicAutos/routineJson",
                "{}");

        System.out.println("Routine JSON: " + routineJson);

        Map<?, ?> root = gson.fromJson(routineJson, Map.class); // {"blockNum"={{"type":"example"}, {params:{}}}}

        for (Map.Entry<?, ?> entry : root.entrySet()) {
            Map<?, ?> block = (Map<?, ?>) entry.getValue();

            String type = (String) block.get("type");
            Map<?, ?> params = (Map<?, ?>) block.get("params");

            Map<String, Integer> paramMap = new HashMap<>();

            for (Map.Entry<?, ?> paramEntry : params.entrySet()) {
                paramMap.put(
                        (String) paramEntry.getKey(),
                        Integer.parseInt((String) paramEntry.getValue()));
            }

            Command step = buildCommand(type, paramMap);

            if (step != null) {
                cmd.addCommands(step);
            } else {
                System.out.println("Unknown command type: " + type); // TODO: test this
            }
        }

        return cmd;
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
