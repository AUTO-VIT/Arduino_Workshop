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

## Codes used to program sensors using arduino

1. Barometric Pressure Sensor
c
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("BMP390 Absolute Pressure Test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  
  float pressure = bmp.pressure / 100.0; // Convert Pa to hPa
  
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");
  
  delay(1000); // Wait 1 second between readings
}


2. Light Sensor (CdS photoresistor)

• Arduino Leonardo is used as the microcontroller.

• The light sensor is set up in a voltage divider with a 10 kΩ resistor . The output from this divider goes to the analog input pin A1 on the Arduino.

• The LED is powered through digital pin 5 on the Arduino. In practice, there should be a current-limiting resistor (e.g., 220 Ω) in series with the LED to protect it from excessive current.

• All devices share a common VCC and GND with the Arduino.
   
c
int pin_CDS = A1;   // pin number of Light Sensor
int pin_LED0 = 5;    // pin number of LED0
void setup() {
  pinMode(pin_CDS, INPUT);       // Set  light sensor pin as an input pin

  pinMode(pin_LED0, OUTPUT);        // Set LED0 pin as output pin
  // Serial Setting : Baud rate 115200, data 8bit, no parity, stop 1bit
  Serial.begin(115200);    // same as Serial.begin(115200, SERIAL_8N1)
  while(!Serial);          //Wait until the serial port is connected
}
void loop() {
  uint16_t ADC_data;
  // Read the analog value of the light sensor as a digital value
  // The higher the digital value, the brighter the light
  ADC_data = analogRead(pin_CDS);
  Serial.print("ADC Data : ");
  Serial.println(ADC_data);    // Outputs the ADC value in serial 
  // If the value measured by light sensor is less than 512, turn on LED0
  if(ADC_data < 512){
    // Output HIGH to LED0 pin (LED0 ON)
    digitalWrite(pin_LED0, 1);}
  // If the value measured by light sensor is greater than or equal to 512,turn off LED0
  else{
    // Output LOW to LED0 pin (LED0 OFF)
    digitalWrite(pin_LED0, 0);}
delay(500);
}


### Explanation of Code

• The code reads the light sensor’s analog value from pin A1.

• It turns the LED ON if that value is below 512 (indicating lower light) and OFF otherwise.

• The current sensor value is printed to the Serial Monitor for easy observation.

• A small delay is included to slow down the update rate.

<img width="297" height="320" alt="image" src="https://github.com/user-attachments/assets/862a3a91-cc58-4b9b-905f-da84cb221e19" />

| Arduino Pin Number | Light Sensor |
|-------------|--------------|
|  A1   |  CDS   |
|5      | LED0   |

3. DC Motor
   

int pin_DC_A=5;
int pin_DC_B=6;
void setup(){
  pinMode(pin_DC_A, OUTPUT);
  pinMode(pin_DC_B, OUTPUT);}
void loop(){
  int Speed_value;
  for(Speed_value=0;Speed_value<170;Speed_value++){
    forwardRotation(Speed_value);
    delay(20);}
    
  Stop();
  delay(7000);
  
  for(Speed_value=200;Speed_value<0;Speed_value--){
    reverseRotation(Speed_value);
    delay(20);}
  Stop();
  delay(7000);
}

void forwardRotation(int Speed)
{
  analogWrite(pin_DC_A,Speed);
  digitalWrite(pin_DC_B,0);
}
void reverseRotation(int Speed)
{
  analogWrite(pin_DC_B,Speed);
  digitalWrite(pin_DC_A,0);
}
void Stop(void)
{
  digitalWrite(pin_DC_A,0);
  digitalWrite(pin_DC_B,0);
}


4. Smart Tap Automation System
   

#include <Servo.h>
int pin_S = 13;
int pin_PSD = A0;
Servo SERVO;


void setup() {
    Serial.begin(115200);
    pinMode(pin_PSD,INPUT);
    SERVO.attach(pin_S);
    while(!Serial);
}

void loop() {
  long int Dist, ADC_data, Volt;
  char *message;
  ADC_data = analogRead(pin_PSD);
  Dist = 10000 / (ADC_data*434*5/1023-46);
  if(Dist>80){
    Dist=80;
  }
  else if(Dist<10){
    Dist=10;
  }
  if(Dist<15){
    SERVO.write(0);
    message="Tap is open";
  }
  else{
    SERVO.write(180);
     message="Tap is Closed";
  }
  Serial.print("ADC Data:");
  Serial.print(ADC_data);
  Serial.print(", Dist:");
  Serial.print(Dist);
  Serial.print("[Cm] , ");
  Serial.println(message);
  delay(500);

}

5. Servo Motor


   
#include <Servo.h>
int pin_SERVO = 13;
Servo SERVO;

void setup(){
  SERVO.attach(pin_SERVO);
  
  SERVO.write(0);
  delay(2000);
  }

void loop(){
  unsigned char angle;
  for(angle=0;angle<=180;angle+=5){
    SERVO.write(angle);
    delay(500);
    }
   delay(1000);
     for(angle=180;angle>=90;angle-=10){
    SERVO.write(angle);
    delay(1000);
   }
   delay(1000);
   SERVO.write(0);
   delay(1000);
  } 



6. Temperature Sensor


#include <DHT.h>

// Define pin numbers
#define DHTPIN 7      // DHT11 data pin connected to digital pin 7
#define MOTOR_PIN_1 5  // Motor positive terminal connected to pin 5
#define MOTOR_PIN_2 6  // Motor negative terminal connected to pin 6

#define DHTTYPE DHT11  // Define sensor type

DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor

// Define threshold temperature (in Celsius) 
#define TEMPERATURE_THRESHOLD 37

void setup() {
  // Start the serial communication for debugging
  Serial.begin(9600);
  
  // Initialize the DHT sensor
  dht.begin();
  
  // Set motor pins as OUTPUT
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  
  // Start with the motor turned off
  digitalWrite(MOTOR_PIN_1, LOW);
  digitalWrite(MOTOR_PIN_2, LOW);
}

void loop() {
  // Read the temperature from the DHT sensor
  float temperature = dht.readTemperature(); 
  
  // Check if the reading is valid
  if (isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Output temperature value to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  // Check if the temperature exceeds the threshold
  if (temperature >= TEMPERATURE_THRESHOLD) {
    Serial.println("Threshold reached! Activating motor...");
    
    // Turn on the motor for 5 seconds
    digitalWrite(MOTOR_PIN_1, HIGH);  // Set positive motor pin to HIGH
    digitalWrite(MOTOR_PIN_2, LOW);   // Set negative motor pin to LOW
    
    delay(5000);  // Wait for 5 seconds
    
    // Stop the motor
    digitalWrite(MOTOR_PIN_1, LOW);   // Turn off the motor
    digitalWrite(MOTOR_PIN_2, LOW);   // Turn off the motor
    
    Serial.println("Motor stopped!");
  }

  // Wait for 1 second before checking the temperature again
  delay(1000);  // Wait for 1 second before reading the temperature again
}



7. Temperature Controlled DC Motor

## Circuit Diagram

<div align="center">
   
![image](https://github.com/user-attachments/assets/6b0c1499-4af1-4804-b85c-3e2c20aad16d)
</div>

## Explanation of Circuit
- An *Arduino* serves as the main controller.  
- The *temperature sensor* (LM35) is connected to an analog input pin (e.g., A0). This pin reads the voltage output of the sensor, which corresponds to the measured temperature.  
- A *motor driver* is used to power the *DC fan*. The Arduino’s digital output pin controls the motor driver, switching the fan power on or off. 
- The Arduino, sensor, and driver circuit share a *common ground* (GND). If the fan requires a higher voltage (e.g., 12V), an external power supply is provided, and its negative terminal is tied to the Arduino’s GND.
  

int pin_TEMP = A4;
int pin_DC_A = 5;
int pin_DC_B = 6;
void setup(){
  pinMode(pin_TEMP, INPUT);
  pinMode(pin_DC_A, OUTPUT);
  pinMode(pin_DC_B, OUTPUT);
  pinMode(pin_TEMP, INPUT);
  while(!Serial);
}


void loop() {
  long int Temp, ADC_data;
  //Reads the analog value of temperature sensor as 10-bit digital value
  ADC_data=analogRead(pin_TEMP);
  //Convert digital value to temperature value
  //Temp=Voltage[V]*100
  Temp = ADC_data * 100 * 5/ 1023;
  //Outputs "ADC Data: " serially
  Serial.print("ADC_data:");
  //Output the measured value of temperature sensor serially
  Serial.print(ADC_data);
  //Outputs ", Temp :" serially
  Serial.print(", Temp :");
  //Outputs temperature value serially
  Serial.print(Temp);
  //outputs "[C]" serially
  Serial.println("[C]");
  delay(500);
  if(Temp<31)
  {
    forwardRotation(100);
  }
  else if(Temp>31)
  {
    reverseRotation(100);
  }
  else{
    Stop();
    }
    delay(100);
  }
  void forwardRotation(int Speed){
    analogWrite(pin_DC_A, Speed);
    digitalWrite(pin_DC_B, 0);
  }
  void reverseRotation(int Speed){
    analogWrite(pin_DC_B, Speed);
    digitalWrite(pin_DC_A, 0);
  }
  void Stop(void){
    digitalWrite(pin_DC_A, 0);
    digitalWrite(pin_DC_B, 0);
  }

## Explanation of Code
- The code *reads the sensor’s analog voltage* using analogRead().  
- It *converts* this reading to a temperature value (depending on the specific sensor calibration).  
- The temperature is then *compared to a threshold* (e.g., 35°C).  
- If the measured temperature *exceeds the threshold, the Arduino **sets a digital output pin HIGH*, powering the transistor and turning the fan on. Otherwise, it sets the pin LOW, turning the fan off.  
- Throughout the loop, the *temperature and fan status* can be printed to the Serial Monitor for debugging or monitoring purposes.

8. Gas Sensor


const int sensor_pin = A0; 
int red =9;
int green =10;
void setup() {
 Serial.begin(115200);
  pinMode(sensor_pin, INPUT); 
 pinMode(red,OUTPUT);
  pinMode(green,OUTPUT);}
 
void loop() {
 
 int sensor_analog;
 sensor_analog = analogRead(sensor_pin);
 Serial.println(sensor_analog);
   if(sensor_analog < 390){
    // Output HIGH to LED0 pin (LED0 ON)
    digitalWrite(green, 1);
    digitalWrite(red, 0);}
  // If the value measured by light sensor is greater than or equal to 512,turn off LED0
  else{
    // Output LOW to LED0 pin (LED0 OFF)
    digitalWrite(green, 0);
    digitalWrite(red, 1);}
 delay(1000);
}


9. Moisture Sensor


const int sensor_pin = A0;
 Serial.begin(115200); }
 
void loop() {
 float moisture_percentage;
 int sensor_analog;
 sensor_analog = analogRead(sensor_pin);
 moisture_percentage = ( 100 - ( (sensor_analog/1023.00) * 100 ) );
 Serial.print("Moisture Percentage = ");
  Serial.print(moisture_percentage);
  Serial.print(", analog = ");
 Serial.println(sensor_analog);
 delay(1000);
}    


10. Potentiometer


#include <Servo.h>
int pot=A3;
Servo myServo;
void setup() {
  pinMode(pot,INPUT);
  myServo.attach(13);
  delay(2000);

}
void loop(){
  long int Volt;
  Volt=analogRead(pot);
  Volt=map(Volt,120,500,0,180);
   
  myServo.write(Volt);
  Serial.print("Potentio value: ");
  Serial.print(analogRead(pot));
  Serial.print("  servo angle: ");
  Serial.println(Volt);
  delay(1000);
}


11. DC Motor Speed & Direction Control

## Circuit Diagram
![image](https://github.com/user-attachments/assets/f6b4486d-6597-4626-afb9-31aa2b91765b)

## Explanation of Circuit
- *Microcontroller (Arduino)*: Provides the control signals for motor speed and direction.  
- *Motor Driver IC (e.g., L293D or L298N)*:
  - *Input Pins* (IN1, IN2, IN3, IN4) connect to Arduino digital pins. Two of these pins control the direction (e.g., IN1 and IN2 for one motor), while one (or both) can be used with *PWM* for speed control on Arduino pins supporting analogWrite().
  - *Enable Pins* (EN1, EN2) may need to be connected to a digital pin or tied HIGH for the driver to operate.
  - *Motor Power Supply (Vs)* provides the voltage and current for the DC motor (e.g., 9V or 12V).  
  - *Logic Power (Vss)* powers the driver’s internal logic (usually 5V from the Arduino).  
- *DC Motor* is connected across the driver’s *motor output pins* (e.g., OUT1 and OUT2 for a single motor).  
- *Common Ground*: The negative terminal of the motor’s power supply and the Arduino GND are tied together to ensure the signals share a common reference.

## Code
c
int pin_DC_A=5;
int pin_DC_B=6;
void setup(){
  pinMode(pin_DC_A, OUTPUT);
  pinMode(pin_DC_B, OUTPUT);}
void loop(){
  int Speed_value;
  for(Speed_value=0;Speed_value<170;Speed_value++){
    forwardRotation(Speed_value);
    delay(20);}
    
  Stop();
  delay(7000);
  
  for(Speed_value=200;Speed_value<0;Speed_value--){
    reverseRotation(Speed_value);
    delay(20);}
  Stop();
  delay(7000);
}

void forwardRotation(int Speed)
{
  analogWrite(pin_DC_A,Speed);
  digitalWrite(pin_DC_B,0);
}
void reverseRotation(int Speed)
{
  analogWrite(pin_DC_B,Speed);
  digitalWrite(pin_DC_A,0);
}
void Stop(void)
{
  digitalWrite(pin_DC_A,0);
  digitalWrite(pin_DC_B,0);
}


## Explanation of Code
- *Pin Assignments*:  
  - Two Arduino pins are designated for motor direction (e.g., dirPin1 and dirPin2).  
  - One Arduino PWM pin is designated for motor speed (e.g., pwmPin).  
- *Setup*:  
  - The direction pins are declared as outputs using pinMode().  
  - The PWM pin is also set as an output.
- *Loop*:  
  - *Direction Control*: Set one direction pin HIGH and the other LOW to spin the motor one way; reverse their states to spin in the opposite direction.  
  - *Speed Control*: Use analogWrite(pwmPin, speedValue); to vary the duty cycle (0–255), controlling how fast the motor spins.  
- *Example Flow*:  
  1. *Set direction* pins to define clockwise or counterclockwise rotation.  
  2. *Write a PWM value* (e.g., between 0 and 255) to the speed pin to run the motor at the desired speed.  
  3. (Optional) *Delay* to observe the motor’s behavior before changing direction or speed again.  

By adjusting the PWM duty cycle, you regulate the motor’s speed; by switching the direction pins, you reverse the motor’s rotation. Together, these steps form a basic but effective method for *DC motor speed and direction control* using a standard motor driver IC and an Arduino.

## proof of working
![WhatsApp Image 2025-03-05 at 14 04 40_a2361a84](https://github.com/user-attachments/assets/b263fdab-30df-46d4-a129-3fac51068fdb)
![WhatsApp Image 2025-03-05 at 14 04 43_1f8acbd7](https://github.com/user-attachments/assets/68c180d5-ab0d-40aa-a085-291f7fafa030)


## Note

| Arduino Pin Number | Light Sensor |
|-------------|--------------|
|  5   |  AIN   |
