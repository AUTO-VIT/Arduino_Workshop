# Arduino Workshop Report

---

## Introduction

This workshop was designed to introduce the fundamentals of embedded electronics using Arduino.  
Explaining how microcontrollers differ from microprocessors, explored the Arduino UNO’s anatomy, and got hands-on coding experience.  
Enabling building simple circuits, writing basic sketches in the Arduino IDE, and understand common communication protocols.

---

## Microprocessors vs. Microcontrollers

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

---

## Pulse Width Modulation (PWM)

PWM toggles a digital pin between HIGH and LOW to create an average voltage level.  
Duty cycle defines the percentage of time the signal is HIGH in each cycle.  
Common uses include dimming LEDs, controlling motor speed, and generating audio tones.

---

## Communication Protocols

We covered three serial protocols:

### UART (Serial)

- Full-duplex, asynchronous  
- Two wires: TX (transmit) and RX (receive)  
- Configurable baud rate (e.g., 9600 bps)  

### I2C

- Two-wire, half-duplex: SDA (data), SCL (clock)  
- Multi-master, multi-slave  
- Each device has a unique 7-bit address  

### SPI

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
- Light (LDR)  
- Ultrasonic distance (HC-SR04)  

### Actuators  
- DC motors  
- Servo motors  
- Buzzer  

### Drivers  
- Transistors (e.g., TIP120) for switching high currents  
- Motor driver ICs (e.g., L298N) to control direction and speed  

---


## Arduino IDE & Coding Structure

All sketches follow this template:

``cpp
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
