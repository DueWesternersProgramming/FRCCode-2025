import sys
import time
from networktables import NetworkTables
import main


NetworkTables.initialize(server="127.0.0.1")  # Replace TEAM with your team number
data = NetworkTables.getTable("AutomationData")
time.sleep(1)  # Time to connect fully
if data.getNumber("Reef Side", -1) != -1:
    print("CONNECTED")
else:
    NetworkTables.initialize(
        server="roborio-8575-frc.local"
    )  # Replace TEAM with your team number
    data = NetworkTables.getTable("AutomationData")
    time.sleep(1)  # Time to connect fully
    if data.getNumber("Reef Side", -1) == -1:
        print("IMPORTANT: COULD NOT CONNECT/RETRIVE VALUES")
        import error_window


def getNum(key):

    return data.getNumber(key, 1.0)


def setNum(key, value):
    print(f"Setting {key} to {value}")
    data.putNumber(key, value)
