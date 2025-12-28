package frc.robot.configurableAutos;

import java.util.List;

/**
 * Definition of a dynamic autonomous command, including its name, parameters,
 * and
 * factory.
 * 
 * @param name    The name of the command. String name
 * @param params  The list of parameters for the command. List<AutoParamDef>
 * @param factory The factory to create the command. List<AutoParamDef>
 */
public record AutoCommandDef(
        String name,
        List<AutoParamDef> params,
        AutoCommandFactory factory) {
}
