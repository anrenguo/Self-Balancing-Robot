# Hardware Bill of Materials (BOM)

| Component | Specification / Model | Qty | Role in System |
| :--- | :--- | :--- | :--- |
| **Microcontroller** | Arduino Uno R3 | 1 | Handles I2C sensor polling and executes the PID control loop. |
| **IMU Sensor** | MPU6050 | 1 | 6-DoF Accelerometer and Gyroscope for calculating tilt angle. |
| **Motor Driver** | L298N Dual H-Bridge | 1 | Isolates high-current motor power from the logic board. |
| **Motors** | 1:48 "TT" DC Gear Motors (3-6V) | 2 | Provides primary wheel actuation and torque for balancing corrections. |
| **Wheels** | 65mm Rubber Tread Wheels | 2 | Press-fits onto the TT motors to provide floor traction. |
| **Power Supply** | 18650 Li-ion Cells (3.7V) | 2 | Wired in series (7.4V nominal) to power motors and logic. |
| **Battery Holder** | 2-Slot 18650 Battery Sled | 1 | Secures the cells to the chassis and routes series power output. |
| **Power Distribution** | Wago 221-413 Splicing Connectors | 2 | Splits the 7.4V battery rail to the L298N and Arduino. |
| **Logic Power Adapter** | 5.5mm x 2.1mm DC Barrel Jack | 1 | Routes the split 7.4V power cleanly into the Arduino. |
| **Tuning Interface** | 10kΩ Rotary Potentiometer | 1 | Analog input for real-time, on-the-fly tuning of the angular setpoint. |
| **Breadboard** | 170-Tie Point Breadboard | 1 | Enables wiring of the MPU6050 and Potentiometer. |
| **Chassis** | 5mm PVC Board / Custom Cut | 1 | Rigid structural base for mounting components. Custom cut using utility knife and assembled with hot glue. |
