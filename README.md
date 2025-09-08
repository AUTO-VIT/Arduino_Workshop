# Arduino Workshop Report

---

## Introduction

This workshop was designed to introduce the fundamentals of embedded electronics using Arduino.  
Explaining how microcontrollers differ from microprocessors, explored the Arduino UNO’s anatomy, and got hands-on coding experience.  
Enabling building simple circuits, writing basic sketches in the Arduino IDE, and understand common communication protocols.

---

## Microprocessors vs. Microcontrollers

<img width="550" height="313" alt="image" src="https://github.com/user-attachments/assets/7b842433-f7de-4093-ba27-78cb4fd2bc09" />

A clear understanding of these two classes of chips helps in choosing the right platform.

| Feature                | Microprocessor               | Microcontroller                     |
|------------------------|------------------------------|-------------------------------------|
| Integration            | CPU only                     | CPU + RAM + I/O ports on one chip   |
| Use Case               | General-purpose computing    | Dedicated tasks in embedded systems |
| Cost                   | Higher                       | Lower                               |
| Power Consumption      | Higher                       | Lower                               |
| Examples               | Intel Core, AMD Ryzen        | Atmel AVR (Arduino), PIC            |

---

## What Is a Microcontroller?

A microcontroller is a “tiny computer on a single chip” programmed for specific tasks.  
It integrates a processor core, memory (RAM/Flash), and peripherals such as timers, GPIO, ADCs, and communication interfaces.  
Unlike microprocessors, microcontrollers excel at real-time control and low-power applications.

---

## Why Arduino?

<img width="540" height="610" alt="image" src="https://github.com/user-attachments/assets/47371bc7-7f5b-43d6-ba29-5948a0a719ad" />

Arduino was chosen for its:  
- Open-source electronics platform and schematics  
- Beginner-friendly IDE and simplified programming model  
- Vast global community offering tutorials and libraries  
- Affordable boards and accessories  
- Cross-platform support (Windows, macOS, Linux)  
- Availability of online simulators (e.g., Tinkercad) for virtual prototyping  

---

## Anatomy of the Arduino UNO

The UNO board comprises:  
- ATmega328P microcontroller  
- 14 digital I/O pins (6 PWM outputs)  
- 6 analog input pins  
- 16 MHz crystal oscillator  
- USB interface for programming and power  
- Power jack (7–12 V input) and voltage regulator  
- Reset button and onboard LED on pin 13

  <img width="576" height="793" alt="image" src="https://github.com/user-attachments/assets/37df5fe8-b07d-4ae1-8867-208dd399efcb" />

---

## Basic Components

Before wiring circuits, juniors should recognize these parts:  
- Resistors: Limit current flow  
- Capacitors: Store and filter charge  
- Breadboard: Solderless platform for building and testing circuits  
- Jumper wires: Connect components quickly  

---

## Types of Input and Output

Arduino pins handle:  
- Digital I/O: HIGH (5 V) or LOW (0 V)  
- Analog input: Reads voltage between 0–5 V via ADC (0–1023)  
- PWM output: Simulates analog voltage by switching digital pins at varying duty cycles

  <img width="527" height="854" alt="image" src="https://github.com/user-attachments/assets/96b6ccba-a8a5-4a44-81d8-66bfa73c22f5" />

---

## Pulse Width Modulation (PWM)

PWM toggles a digital pin between HIGH and LOW to create an average voltage level.  
Duty cycle defines the percentage of time the signal is HIGH in each cycle.  
Common uses include dimming LEDs, controlling motor speed, and generating audio tones.

<img width="572" height="718" alt="image" src="https://github.com/user-attachments/assets/f3dae23b-34e2-462a-ae6a-103cdd2b9b84" />

---

## Communication Protocols

We covered three serial protocols:

### UART (Serial)

<img width="525" height="512" alt="image" src="https://github.com/user-attachments/assets/5e68f776-9ae2-485c-9bab-8f0e7e253cde" />

- Full-duplex, asynchronous  
- Two wires: TX (transmit) and RX (receive)  
- Configurable baud rate (e.g., 9600 bps)  

### I2C

<img width="512" height="591" alt="image" src="https://github.com/user-attachments/assets/39d09816-6579-42d1-ae01-98cbd6f25d6a" />

- Two-wire, half-duplex: SDA (data), SCL (clock)  
- Multi-master, multi-slave  
- Each device has a unique 7-bit address  

### SPI

<img width="593" height="608" alt="image" src="https://github.com/user-attachments/assets/032848c7-6ae3-4716-be12-dbdf0ad0fcf3" />

- Four-wire, full-duplex: MOSI, MISO, SCK, SS (chip select)  
- One master, multiple slaves  
- High data rates, ideal for fast peripherals  

#### Comparison

| Protocol | Wires                | Master/Slave          | Speed            | Typical Use Cases             |
|----------|----------------------|-----------------------|------------------|-------------------------------|
| UART     | 2 (TX/RX)            | Point-to-point        | Up to 1 Mbps     | Serial consoles, GPS modules  |
| I2C      | 2 (SDA/SCL)          | Multi-master/slave    | Up to 400 kbps   | EEPROMs, sensors              |
| SPI      | 4 (MOSI/MISO/SCK/SS) | Master/slave          | Up to tens of Mbps | SD cards, displays          |

---

## Basic Sensors, Actuators, and Drivers

### Sensors  
- Temperature (e.g., LM35)

  <img width="300" height="1000" alt="image" src="https://github.com/user-attachments/assets/72ad7254-435f-45d9-b4fd-f068e21cf581" />
- Light (LDR)
   
  <img width="352" height="1852" alt="image" src="https://github.com/user-attachments/assets/ea636d5a-60b1-4b95-97ac-0a1fac82a206" />
- Ultrasonic distance (HC-SR04)
  
  <img width="300" height="2380" alt="image" src="https://github.com/user-attachments/assets/6d32d0c4-40fe-4e8d-a726-70d6c6603d22" />

### Actuators  
- DC motors
  
  <img width="300" height="1373" alt="image" src="https://github.com/user-attachments/assets/1550aeb5-5c06-4c34-a483-b91e06d22329" />
- Servo motors
  
  <img width="300" height="1254" alt="image" src="https://github.com/user-attachments/assets/d1996828-8619-4c4b-b219-8b0aa4099b13" />
- Buzzer
  
  <img width="350" height="1250" alt="image" src="https://github.com/user-attachments/assets/9512cb51-5e46-4974-924f-3a91ffa9c518" />

### Drivers  
- Transistors (e.g., TIP120) for switching high currents
  
  <img width="315" height="449" alt="image" src="https://github.com/user-attachments/assets/fc67710d-738c-443f-8509-fe5f8678844d" />
- Motor driver ICs (e.g., L298N) to control direction and speed
  
  <img width="350" height="500" alt="image" src="https://github.com/user-attachments/assets/5f7d88c4-0382-47b0-8181-fd60ae91bbbc" />  

---


## Arduino IDE & Coding Structure

All sketches follow this template:

c
void setup() {
  // Runs once at startup
}

void loop() {
  // Repeats indefinitely
}




### Common functions:

pinMode(pin, INPUT/OUTPUT)

digitalWrite(pin, HIGH/LOW)

digitalRead(pin) → HIGH/LOW

analogWrite(pin, 0–255)

analogRead(pin) → 0–1023

delay(ms)

Serial.begin(baudRate)

Serial.println(value)




## Example: Blink LED

void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}



## Practice and Simulation

Use Tinkercad’s Circuits simulator for virtual prototyping

Experiment with provided code examples before wiring physical setups

Explore online resources and community projects


## Sample Projects
Blinking LED – Learn digital I/O and timing

Temperature Monitor – Read analog sensor and display on Serial Monitor

Motor Control – Drive a DC motor with PWM and L298N driver


### Troubleshooting Tips
Check wiring: Loose or reversed connections cause failures

Verify pin numbers in code match hardware

Use Serial Monitor for debugging sensor values

Ensure correct board and COM port selected in IDE

Add delays when reading sensors to allow signal stabilization


## Additional Resources

Arduino Official Docs: https://www.arduino.cc/reference/

Tinkercad Circuits: https://www.tinkercad.com/circuits

Community Forum: https://forum.arduino.cc/

Electronics Tutorials: https://www.allaboutcircuits.com/

## Example References

1. Barometric Pressure Sensor
2. Light Sensor (CdS photoresistor)
3. DC Motor
4. Smart Tap Automation System
5. Servo Motor
6. Temperature Sensor
7. Temperature Controlled DC Motor
8. Gas Sensor
9. Moisture Sensor
10. Potentiometer
11. DC Motor Speed & Direction Control
12. Reference Material

https://github.com/KL-Mithunvel/Sensor_LAB
