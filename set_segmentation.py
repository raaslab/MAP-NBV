import math
import airsim
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()

#success = client.simSetSegmentationObjectID("Cylinder_2", 50);
#print(success)

success = client.simSetSegmentationObjectID(".*", 80, True);
print(success)

success = client.simSetSegmentationObjectID("767_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("a340_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("concorde_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("sr71_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("hercules_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("c17_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("patrol_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("cruise_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("boat_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("mountaintrain_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("electrictrain_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("Landscape_[\w]*", 80, True);
print(success)

success = client.simSetSegmentationObjectID("private_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("747_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("diesel_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("silo_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("bigben_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("church_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("clock_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("saturnv_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("atlas_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("v2_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("sparrow_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("pylon_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("maverick_[\w]*", 20, True);
print(success)

success = client.simSetSegmentationObjectID("revolver_[\w]*", 20, True);
print(success)
