# AMS AS5048B - Linux Library for 14-bit Magnetic Rotary Position Sensor

[![CI](https://github.com/oliexdev/openScale/actions/workflows/ci.yml/badge.svg)](https://github.com/oliexdev/openScale/actions/workflows/ci.yml)  
[![Translation Status](https://hosted.weblate.org/widgets/openscale/-/strings/svg-badge.svg)](https://hosted.weblate.org/engage/openscale/?utm_source=widget)  

## Overview  
This repository provides a **Linux library** for interfacing with the **AMS AS5048B**, a **14-bit I2C magnetic rotary position sensor**. The AS5048B is designed for high-precision **angle measurement** applications, making it ideal for robotics, automation, motor control, and other **position-sensing** needs.  

---

## Features 🚀  
✅ **14-bit High-Resolution** – Accurate angular position sensing.  
✅ **I2C Interface** – Communicates seamlessly with Linux-based systems.  
✅ **360° Absolute Positioning** – No need for an external reference.  
✅ **Configurable Zero Position** – Supports offset correction for custom applications.  
✅ **Error Detection** – Built-in diagnostic features to detect failures.  

---

## Installation 🛠️  
Clone the repository and install dependencies:  
```bash
git clone https://github.com/YOUR_USERNAME/AS5048B-Library.git  
cd AS5048B-Library  
make  
sudo make install  
```

---

## Usage 📖  
Example code to read the angle from AS5048B:  
```cpp
#include "AS5048B.h"

AS5048B sensor(0x40);  // I2C address

int main() {
    sensor.begin();
    float angle = sensor.getAngle();
    printf("Current Angle: %.2f°\n", angle);
    return 0;
}
```

---

## Applications 🔄  
- **Robotics & Automation** 🤖  
- **Motor Position Control** ⚙️  
- **Industrial Motion Tracking** 🏭  
- **Gimbal & Camera Stabilization** 🎥  
- **Aerospace & Defense** ✈️  

---

## License 📜  
This project is licensed under **GPL v3**, ensuring open-source collaboration and improvements.  

💡 **Contributions Welcome!** If you’d like to improve this library, feel free to submit a **pull request** or report **issues**. 🎯
