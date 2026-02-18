# Autonomous Drone Landing using ArUco Marker in Webots

---

##  Project Structure
```
drone_landing_simulation/
│
├── controllers/
│       └── drone_controller/
│                 └── drone_controller.py
│                 └── aruco.py
│                 └── config.py
│                 └── controller.py
│                 └── motors.py
│                 └── navigation.py
│                 └── sensors.py
│                 └── utils.py
│
├── worlds/
│       └── Drone Landing Simulation.wbt
│
├── requirements.txt
│
└── README.md
```
---

## System Requirements

- Windows / Linux / macOS
- Webots R2023 or later
- Python 3.8+
- Minimum 8 GB RAM recommended

---

## How to Download and Run

### Step 1: Download the Project

Option 1: Download ZIP

1. Click the green **Code** button on GitHub  
2. Click **Download ZIP**  
3. Extract the ZIP file  

Option 2: Using GitHub Desktop

1. Open GitHub Desktop  
2. Click **File → Clone repository**
3. Select this repository  
4. Click **Clone**

---

### Step 2: Install Webots

Download Webots from:

https://cyberbotics.com/

Install and open Webots.

---

### Step 3: Install Python Dependencies
```
pip install -r requirements.txt
```
---

### Step 4: Open Project in Webots

1. Open Webots  
2. Click:
```
File → Open World
```

3. Navigate to:
```
drone_landing_simulation/worlds/
```

4. Open:

```
Drone Landing Simulation.wbt
```

---


## License

This project is for educational and research purposes.

---
