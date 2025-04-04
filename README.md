# AI-based-Ambulance-detcetion
Ai model will control the traffic signal
This project implements an intelligent traffic signal system that prioritizes ambulances in heavy traffic using a combination of camera-based detection, siren recognition, and Arduino-based control. The system ensures that when an ambulance approaches an intersection, the traffic signal automatically turns green for its lane, reducing delay and improving emergency response time.

The system is Arduino Mega-based and uses camera vision and siren sound recognition to detect ambulances. It is designed for a four-lane road divided into two-lane sections, ensuring that only the affected lane's signal changes, without disrupting other lanes.

Key Features
✅ Camera-based ambulance detection – Uses image processing to recognize ambulances in real time
✅ Siren sound recognition – Detects emergency sirens using an audio processing module
✅ Automatic traffic light control – Turns green when an ambulance is detected, returning to normal afterward
✅ Multi-lane management – Works on a four-lane road divided into two sections
✅ Priority handling – If two ambulances approach from opposite directions, the closer one gets priority
✅ Wokwi-based simulation – Uses Wokwi for simulating the Arduino-based system
✅ Arduino Mega integration – Ensures real-time signal processing and control

System Architecture
1️⃣ Detection System (Camera & Microphone)
Camera Vision Module

Detects an ambulance based on pre-trained ML models (e.g., OpenCV with Haar Cascades or YOLO).

Processes video frames to check for an ambulance pattern on the road.

Communicates with the Arduino Mega when an ambulance is detected.

Siren Recognition Module

Uses a microphone with an FFT-based sound recognition algorithm to detect ambulance sirens.

Filters out background noise and only responds to specific frequency ranges of sirens.

Sends a signal to Arduino when an emergency sound is detected.

2️⃣ Control System (Arduino Mega)
Traffic Light Control Algorithm

Normal operation: Follows pre-programmed traffic light cycles.

Ambulance detected:

Turns the respective lane’s signal green to allow passage.

Keeps other signals unchanged to avoid unnecessary congestion.

After the ambulance passes, the signal resets to normal operation.

Priority Handling at Intersections

If two ambulances approach from different directions:

The closer ambulance gets the green signal first.

Once it passes, the second ambulance’s path is cleared.

3️⃣ Hardware Components
Arduino Mega 2560 – Central controller for signal processing

Camera Module (ESP32-CAM / Raspberry Pi Camera / USB Camera) – Captures real-time traffic footage

Microphone Module – Detects ambulance siren

LED Traffic Lights (Red, Yellow, Green) – Simulates real-world traffic lights

Servo Motor (Optional) – Can be used for physical barriers to open for ambulances

4️⃣ Software & Tools Used
Arduino IDE – Code deployment on Arduino Mega

Wokwi – Simulating the traffic system

Python (OpenCV + YOLO) – Ambulance detection via camera

MATLAB (Optional) – For signal processing of siren detection

Keil IDE (Optional) – If extending the project for ARM microcontrollers

Working Mechanism
1️⃣ Ambulance Detection (Camera & Sound)

The camera continuously scans traffic using OpenCV.

The microphone listens for siren frequencies.

2️⃣ Signal Processing (Arduino Mega)

If an ambulance is detected:

The corresponding lane gets a green signal.

Other lanes remain unaffected.

If two ambulances approach:

The closer one gets priority.

3️⃣ Traffic Light Control

Once the ambulance passes, the signal returns to normal.

If another ambulance is approaching, the system rechecks priority.

Future Improvements
🚀 RFID-Based Ambulance Detection – Instead of relying on cameras, use RFID tags in ambulances for direct recognition.
🚀 V2X Communication – Enable direct communication between ambulances and traffic signals.
🚀 AI-Based Traffic Prediction – Implement machine learning models to dynamically adjust traffic light timing.

Setup Instructions
Hardware Setup

Connect the camera and microphone modules to the Arduino Mega.

Wire up the LED traffic lights for simulation.

Software Installation

Upload the Arduino code for traffic control.

Run the Python script for camera-based detection.

If using MATLAB, implement siren detection.

Simulation (Wokwi)

Open Wokwi and load the Arduino Mega simulation.

Test different traffic scenarios.
