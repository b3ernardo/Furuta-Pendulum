# Furuta Pendulum

Implementation of the **Furuta Pendulum** project, developed as part of my undergraduate thesis in Control and Automation Engineering.  
The goal is to model, simulate, and implement real-time control of a Furuta Pendulum using an **ESP32** microcontroller.

---

## 📂 Repository Structure

- **/arduino_code** → Control algorithm implemented on the ESP32 (Arduino IDE).  
- **/3d_print** → STL files for 3D printing the physical structure.  
- **/matlab_simulink** → MATLAB scripts for modeling, analysis, and Simulink models for simulation and validation.  
- **/python_code** → Jupyter notebooks for data acquisition from the ESP32 via serial communication.  
- **3D CAD Online** → [View on Onshape](https://cad.onshape.com/documents/d0d2d40c9dd0c88f858cd038/w/86dce8532c9b6c8310216a8c/e/c2fa23f7c1c4edc232f4e580?renderMode=0&uiState=68e07acc73f9dd32bc20c706)  

---

## 🛠️ Features

- State-space modeling and linearization of the Furuta Pendulum.  
- LQR (Linear Quadratic Regulator) control design.  
- Real-time implementation on ESP32 using PWM and rotary encoders.  
- Complete 3D-printable structure for assembling the physical prototype.

---

## ⚙️ How It Works

1. The ESP32 runs the control loop at 100 Hz (Ts = 10 ms), reading angular positions and velocities from encoders.  
2. The control signal is computed via LQR feedback and applied to the motor driver using PWM modulation.  
3. Disturbances (step or pulse) can be injected to evaluate controller performance.  
4. The ESP32 sends sampled data through the serial interface, which is logged by the Python script.  
5. The collected data can then be analyzed and compared with Simulink simulations.

---

## 🎯 Purpose

This project was developed as an **educational platform for Control Engineering**, combining theoretical modeling, simulation, and practical implementation.  
It enables students and enthusiasts to reproduce the experiment, understand nonlinear dynamics, and test advanced control techniques in a real embedded environment.

---

## 🧠 Author

Developed by **Bernardo Silva**  
Control and Automation Engineering — UFMG  
LinkedIn: [Bernardo Silva](https://www.linkedin.com/in/bernardo-de-souza-silva/)
