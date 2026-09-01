# 3D-Room-MapperThe repository aims to build a device that can map a room from a single position using a rotating LiDAR sensor and a Raspberry Pi.

The idea is to take an RPLiDAR A1, which can scan a 2D plane on its own, and use a stepper motor to rotate it. The 2D points are then converted into 3D points by the Raspberry Pi using the rotation of the stepper motor and basic trigonometry.

The hardware components used in this project were not specifically chosen for this project. A less powerful motor and a cheaper LiDAR could be used instead. The components are being used because I already had them from previous projects. Therefore, I would not recommend copying my hardware setup. Instead, I recommend looking for other repositories that use hardware specifically selected for their respective projects.

The components used are the aforementioned RPLiDAR A1, a NEMA 23 stepper motor, a TB6600 stepper motor driver, a MEAN WELL LRS-35 power supply, and a Raspberry Pi.
