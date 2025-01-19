package frc.robot.automation;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.CowboyUtils;

public class AutomationSelector {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    NetworkTable networkTable = inst.getTable("AutomationData");

    IntegerPublisher reefSide;
    IntegerPublisher position;
    IntegerPublisher level;

    IntegerSubscriber reefSideSub;
    IntegerSubscriber positionSub;
    IntegerSubscriber levelSub;

    public AutomationSelector() {
        reefSide = networkTable.getIntegerTopic("Reef Side").publish();
        position = networkTable.getIntegerTopic("Position").publish();
        level = networkTable.getIntegerTopic("Level").publish();
        reefSide.set(1);
        position.set(1);
        level.set(1);

        reefSideSub = networkTable.getIntegerTopic("Reef Side").subscribe(1);
        positionSub = networkTable.getIntegerTopic("Position").subscribe(1);
        levelSub = networkTable.getIntegerTopic("Level").subscribe(1);

    }

    public int getReefSide() {

        return (int) reefSideSub.get();
    }

    public int getPosition() {
        return (int) positionSub.get();
    }

    public int getHeight() {
        return ((int) levelSub.get());
    }

}
