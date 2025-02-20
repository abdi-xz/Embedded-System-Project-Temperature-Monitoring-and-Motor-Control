# Temperature-Based LED & Motor Control System using PIC16F877A  

## 📌 Project Overview  
This project is a **temperature-controlled LED system** and **motor control unit** using a **PIC16F877A** microcontroller. It utilizes an **LM35 temperature sensor** to activate LEDs based on temperature thresholds and a **L293D motor driver** to control motor operations based on button inputs.  

## 🎯 Features  
- **Temperature Monitoring (LM35 Sensor)**  
  - **Below 15°C** → Red LED ON  
  - **Between 15°C and 35°C** → Yellow LED ON  
  - **Above 35°C** → Green LED ON  
- **Motor Control (L293D Motor Driver)**  
  - Button 1 → Motor moves **Forward**  
  - Button 2 → Motor moves **Fast Forward**  
  - Button 3 → Motor **Reverses**  

## 🛠️ Hardware Components  
- **Microcontroller**: PIC16F877A  
- **Temperature Sensor**: LM35  
- **Motor Driver**: L293D  
- **LEDs**: Red, Yellow, Green  
- **Buttons**: 3 Push Buttons for motor control  
- **Power Supply**: 5V  

## 🖥️ Software & Tools  
- **MPLAB X IDE** (for coding and compilation)  
- **XC8 Compiler** (to compile the C program)  
- **Proteus 8** (for circuit simulation)  

## 🚀 How to Use  
1. **Compile the Code**  
   - Open the `.c` file in **MPLAB X IDE**  
   - Compile using **XC8 Compiler** to generate a `.hex` file  

2. **Simulate in Proteus**  
   - Open `proteus_simulation.pdsprj`  
   - Load the compiled `.hex` file into PIC16F877A  
   - Run the simulation and test motor and LED behavior  

3. **Upload to Microcontroller**  
   - Flash the `.hex` file to PIC16F877A using a PIC programmer  


