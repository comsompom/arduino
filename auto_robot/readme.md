# Autonomous Robot - Arduino Mega 2560

## 📋 Project Overview

This autonomous robot uses an Arduino Mega 2560 to perform a predefined movement sequence with obstacle avoidance and ground detection capabilities. The robot features dual sonar sensors, DC motors, accelerometer position tracking, and audio feedback.

## 🛠️ Equipment List

### Core Components
- **Arduino Mega 2560** - Main microcontroller
- **Adafruit Motor Shield** - DC motor control
- **2x DC Motors** - Left and right wheel drive
- **2x HC-SR04 Ultrasonic Sensors** - Forward obstacle and ground detection
- **GY-291 ADXL345 Accelerometer** - Position and distance tracking
- **Buzzer** - Audio feedback and melody playback

### Power & Connectivity
- **12V Power Supply** - For motor operation
- **I2C Communication** - SDA/SCL for accelerometer
- **Digital/Analog Pins** - For sensors and buzzer

## 🔌 Pin Connections

### Motor Shield Connections
```
Motor Shield → Arduino Mega 2560
M1 (Left Motor)  → Motor1 (AF_DCMotor)
M2 (Right Motor) → Motor2 (AF_DCMotor)
```

### Sonar Sensor Connections
```
Forward Sonar (HC-SR04):
- VCC    → 5V
- GND    → GND
- TRIG   → Pin 30
- ECHO   → Pin 31

Ground Sonar (HC-SR04):
- VCC    → 5V
- GND    → GND
- TRIG   → Pin 32
- ECHO   → Pin 33
```

### Accelerometer Connections (GY-291 ADXL345)
```
ADXL345 → Arduino Mega 2560
- VCC    → 3.3V
- GND    → GND
- SDA    → Pin 20 (SDA)
- SCL    → Pin 21 (SCL)
```

### Buzzer Connection
```
Buzzer → Arduino Mega 2560
- VCC    → Pin 13
- GND    → GND
```

## 🧠 System Logic

### 1. Initialization Sequence
```
1. Serial Communication Setup (9600 baud)
2. I2C Communication Initialization
3. ADXL345 Accelerometer Detection & Calibration (400 samples)
4. Sonar Sensor Pin Configuration
5. DC Motor Initialization & Test
6. Audio Feedback (Initial beep)
7. Caribbean Pirates Melody Playback
8. Position Tracking Reset
```

### 2. Movement Sequence
```
Step 1: Forward 2 meters (5 seconds)
Step 2: Right turn 90° (2 seconds)
Step 3: Forward 50cm (1.25 seconds)
Step 4: Right turn 90° (2 seconds)
Step 5: Forward 2 meters (5 seconds)
Step 6: Stop and report final position
```

### 3. Obstacle Avoidance Logic

#### Detection System
- **Forward Sonar:** Monitors obstacles ahead (< 20cm threshold)
- **Ground Sonar:** Monitors ground/cliff detection (> 15cm threshold)
- **Continuous Monitoring:** Both sensors checked every 100ms

#### Avoidance Strategy
```
1. Obstacle/Hole Detected → Immediate Stop
2. Check Left Side Ground → Turn left 0.5s, measure ground, return
3. Check Right Side Ground → Turn right 0.5s, measure ground, return
4. Decision Logic:
   - Both sides have ground → Choose side with more space
   - Only left has ground → Turn left
   - Only right has ground → Turn right
   - No ground on either side → Safety stop
5. Execute avoidance turn
6. Resume forward movement
```

### 4. Position Tracking (ADXL345)

#### Calibration Process
- **400 Sample Calibration:** Takes 400 readings during setup
- **Offset Calculation:** Determines X, Y, Z calibration offsets
- **Real-time Tracking:** Updates position every 50ms

#### Position Integration
```
Acceleration → Velocity → Position
- Reads calibrated acceleration data
- Integrates acceleration to velocity
- Integrates velocity to position
- Reports X, Y, Z coordinates in meters
```

## 📊 Key Parameters

### Motor Control
- **Motor Speed:** 255 (100% power)
- **Forward Duration:** 5 seconds (2 meters)
- **Short Forward Duration:** 1.25 seconds (50cm)
- **Turn Duration:** 2 seconds (90°)

### Sensor Thresholds
- **Obstacle Threshold:** 20cm (forward sonar)
- **Cliff Threshold:** 15cm (ground sonar)
- **ADXL345 Read Interval:** 50ms
- **Sonar Check Interval:** 100ms

### Audio Feedback
- **Buzzer Pin:** 13
- **Initial Beep:** 494Hz, 500ms
- **Setup Complete:** 1000Hz, 100ms (2x)
- **Caribbean Pirates Melody:** Full theme song

## 🔧 Technical Specifications

### Libraries Used
```cpp
#include <Wire.h>        // I2C communication
#include <AFMotor.h>     // DC motor control
```

### Key Functions
- `initializeADXL345()` - Accelerometer setup and detection
- `calibrateADXL345(int samples)` - 400-sample calibration
- `moveForwardWithObstacleCheck()` - Forward movement with obstacle detection
- `moveForwardShort()` - Short forward movement (50cm)
- `turnRight()` / `turnLeft()` - 90° turns
- `avoidObstacle()` - Smart obstacle avoidance
- `checkLeftSideGround()` / `checkRightSideGround()` - Side ground detection
- `updateADXL345Position()` - Real-time position tracking
- `playCaribbeanPiratesMelody()` - Audio feedback

### Safety Features
- **Immediate Stop:** On obstacle or hole detection
- **Ground Verification:** Before choosing avoidance direction
- **Position Monitoring:** Continuous tracking for navigation
- **Error Handling:** Graceful degradation if sensors fail

## 🎯 Movement Algorithm

### Main Loop Flow
```
1. Update ADXL345 position tracking
2. Reset position for new sequence
3. Execute 5-step movement sequence
4. Report final position and distance
5. Stop motors
```

### Obstacle Response Flow
```
1. Continuous sensor monitoring
2. Obstacle/hole detection
3. Immediate motor stop
4. Side ground assessment
5. Direction decision
6. Avoidance execution
7. Movement resumption
```

## 📈 Performance Metrics

### Timing
- **Total Sequence Time:** ~15-20 seconds
- **Setup Time:** ~10-15 seconds (including calibration)
- **Obstacle Response Time:** < 100ms
- **Position Update Rate:** 20Hz (50ms intervals)

### Accuracy
- **Position Tracking:** ±0.1m (with ADXL345 calibration)
- **Obstacle Detection:** ±2cm (HC-SR04 accuracy)
- **Ground Detection:** ±2cm (HC-SR04 accuracy)
- **Turn Precision:** ±5° (time-based)

## 🚨 Troubleshooting

### Common Issues
1. **ADXL345 Not Found:** Check I2C connections (SDA/SCL)
2. **Motors Not Moving:** Verify motor shield connections
3. **Sonar Errors:** Check trigger/echo pin connections
4. **Buzzer Issues:** Verify pin 13 connection

### Debug Output
- **Serial Monitor:** 9600 baud for real-time status
- **Position Reports:** X, Y, Z coordinates during movement
- **Sensor Readings:** Forward and ground distances
- **Calibration Progress:** ADXL345 setup status

## 🔄 Future Enhancements

### Potential Improvements
- **Gyroscope Integration:** For more precise turning
- **Multiple Sonar Sensors:** For better obstacle mapping
- **GPS Module:** For absolute positioning
- **Camera Module:** For visual obstacle detection
- **Wireless Communication:** For remote monitoring

### Code Optimization
- **PID Control:** For smoother motor control
- **Kalman Filtering:** For improved position accuracy
- **Path Planning:** For complex navigation
- **Machine Learning:** For adaptive behavior

---

## 📝 License

This project is open source and available under the MIT License.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

---

**Created by:** Autonomous Robot Development Team  
**Last Updated:** 2024  
**Version:** 1.0
