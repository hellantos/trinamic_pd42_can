This package is used to control the PD42-1-1270 stepper motor from Trinamic via ros2_canopen.

## Usage

### Setup CANController
- Virtual CANController

    ```bash
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 txqueuelen 1000
    sudo ip link set up vcan0
    ```
- Peak CANController
    
    ```bash
    sudo modprobe peak_usb
    sudo ip link set can0 up type can bitrate 1000000
    sudo ip link set can0 txqueuelen 1000
    sudo ip link set up can0
    ```

### Run the node
- Fake slave: 
    
    ```bash
    ros2 run trinamic_pd42_can mock_launch.launch.py
    ```

- Real hardware: 
    
    ```bash
    ros2 run trinamic_pd42_can real_hw_launch.launch.py
    ```

### Init the device

```bash
ros2 service call /trinamic_pd42/init std_srvs/srv/Trigger
```

### Switch mode to ProfilePosition

```bash
ros2 service call /trinamic_pd42/position_mode std_srvs/srv/Trigger
```

### Set target position

```bash
ros2 service call /trinamic_pd42/target canopen_interfaces/srv/COTargetDouble "{ target: 10.0 }"
```

or

```bash
ros2 run trinamic_pd42_can position_tick_client
```