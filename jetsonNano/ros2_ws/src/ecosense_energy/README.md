# ecosense_energy

This package contains the `energy_monitor_node` which monitors and computes the total energy consumed by the system components.

## Node: energy_monitor_node

This node subscribes to power data and integrates it over time to calculate the total energy consumed (in Joules).

### Subscribed Topics

*   `/general_data` ([interfaces/msg/GeneralData](../interfaces/msg/GeneralData.msg))
    *   `power_batt`: Instantaneous power of the battery.
    *   `power_mg`: Instantaneous power of the left motor.
    *   `power_md`: Instantaneous power of the right motor.
    *   `power_jet`: Instantaneous power of the Jetson board.

### Published Topics

*   `/energy_status` ([interfaces/msg/EnergyStatus](../interfaces/msg/EnergyStatus.msg))
    *   `energy_batt`: Total energy consumed by the battery (Joules).
    *   `energy_left`: Total energy consumed by the left motor (Joules).
    *   `energy_right`: Total energy consumed by the right motor (Joules).
    *   `energy_jetson`: Total energy consumed by the Jetson board (Joules).

### Units

*   **Power**: Watts (W)
*   **Time**: Seconds (s)
*   **Energy**: Joules (J)  (where $E = \int P dt$)

### Usage

Run the node using:

```bash
ros2 run ecosense_energy energy_monitor_node
```
