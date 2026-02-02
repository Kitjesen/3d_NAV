## Robot Driver

This package contains the robot driver interface.

**Note**: The odometry and IMU data are now provided by the SLAM system (fastlio2), which publishes:
- `/Odometry` - Robot odometry in `odom` frame
- TF: `odom → body` transformation

For legacy wheel odometry integration, install dependencies:
```bash
sudo apt install python3-dev python3-pip
sudo pip3 install websocket-client
```

Remember to change the ACCID in the code to your robot's serial number (SN) if needed.
