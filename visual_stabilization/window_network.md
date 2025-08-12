# Windows Firewall

To allow ROS2 communicate between Rapbery Pi and your PC we need to adjust firewall settings:

1. Press `Win + R` to open the Run dialog.
2. Type `control`.
3. Open `Windows Defender Firewall` -> `Advanced settings`

4. Create Inbound rule:
    1. Open `Inbound Rules` tab.
    2. Press `New Rule...` button.
    3. `Rule Type` => `Port`
    4. `Protocol and Ports` => `UDP` and `All local ports`
    5. `Action` => `Allow the connection`
    6. `Profile` => All checked
    7. `Name` => `ROS2` (or any you wish)

4. Create Outbound rule:
    1. Open `Outbound Rules` tab.
    2. Press `New Rule...` button.
    3. `Rule Type` => `Port`
    4. `Protocol and Ports` => `UDP` and `All local ports`
    5. `Action` => `Allow the connection`
    6. `Profile` => All checked
    7. `Name` => `ROS2` (or any you wish)

5. Mirror use Window network in WSL:
    1. Open `C:\User\[YourName]\.wslconfig` file.
    2. Add a new line with `networkingMode=mirrored`.

5. Reload your PC

# ROS2 topics

Note: Even after this change whenever you use `ros2 topic ...` command, you should add `--no-daemon` flag in WSL.

e.g. 

```
ros2 topic list --no-daemon
ros2 topic echo /mavros/vision_pose/pose --no-daemon
```