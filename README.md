# Self-Balancing-Cube

Software for a self-balancing cube. Modified code removing bluetooth functionallities and added compatibility with the ESP8266.

showcase of the original Self balancing cube: [https://youtu.be/AJQZFHJzwt4]()
Original code implementation: [https://github.com/remrc/Self-Balancing-Cube/tree/main]()

## Components

3 DC Motors: [https://es.aliexpress.com/item/1005005585439123.html]()

5 IMUs: [https://es.aliexpress.com/item/1005005281279590.html]()

1 Pack 20 PCB Protoboards: [https://es.aliexpress.com/item/32588853051.html]()

1 pack 10 PCB Protoboard 7x9cm: [https://es.aliexpress.com/item/1005004818919331.html]()

1 pack Buzzer 3,3V: [https://es.aliexpress.com/item/1005003022264282.html]()

5 pack voltage regulators 5V: [https://es.aliexpress.com/item/1005006358036073.html]()

5 packs voltage regulators 3V3 THT:

[https://es.aliexpress.com/item/1005001667326055.html]()

[https://es.aliexpress.com/item/1005006134947908.html]()

1 pack de 50 voltage regulators 3V3 SMD: [https://es.aliexpress.com/item/1005005885801552.html]()

1 pack sockets pins (female): [https://es.aliexpress.com/item/4000523047541.html]()

5 pack headers pins (male): [https://es.aliexpress.com/item/1005005990753794.html]()

1 pack headers pins different colors: [https://es.aliexpress.com/item/1005006186853439.html]()

LIPO: 12V batteries 500-450mAh


## Installation

1. Using Platform.io plugin for VScode to upload the code to the microcontroller, create a new Platform.io project and select the correct microcontroller. Then copy the content of the `src/` directory into the directory with same name in the new project.

2. In `src/main.cpp` choose the correct source code for the microcontroller by choosing one of the include files (first two lines in the file). The ESP8266 implementation is selected by default.

```cpp
//#include "code_esp32/functions_esp32.hpp"
#include "code_esp8266/functions_esp8266.hpp"
```

3. Change the pinout of the circuit if using the ESP8266. The default implementation is for the ESP32.

## Changes from ESP32 to ESP8266

### Pins

The default pinout is for the ESP32, it is required to change the pin connections for the ESP8266 implementation (buzzer and motors).

```cpp
// Pins to change for the ESP8266
#define BUZZER      27
#define VBAT        34

#define BRAKE       26

#define DIR1        4
#define PWM1        32

#define DIR2        15
#define PWM2        25

#define DIR3        5
#define PWM3        18
```

* The MPU6050 module is connected using the I2C protocol so it requires a connection to the SCL and SDA pins of the ESP8266.

* Not all pins are needed for connection. Only power, ground, brake (start/stop) and pwm.

### Calculate offsets

Connect ESP to computer and check output and then get value and set it in the `variable_espXX.hpp` file.

```cpp
// print messages to console with calculated offset values
Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
Serial.print("GyY offset value = "); Serial.println(GyY_offset);
Serial.print("GyX offset value = "); Serial.println(GyX_offset);
```

### PWM from ESP32 to ESP8266

Differences between PWM in ESP32 and ESP8266

```cpp
// PWM range avalible bits
#define TIMER_BIT  8
// Max PWM range for 8 bits 
#define PWM_RANGE 255
#define BASE_FREQ  20000

// Number of bits used for the PWM range
int RANGE_BITS = TIMER_BIT;

void setup() {
    // lecSetup is for ESP32. Set ups frequency and PWM range
    ledcSetup(PWM_CH, BASE_FREQ, RANGE_BITS);
    // ESP8266 requires 2 functions and affects all channels
    // Set up pwm range and frequency
    analogWriteRange(PWM_RANGE);
    analogWriteFreq(BASE_FREQ);
    
    // ESP8266 does not have channels
    // set pin to a PWM channel in ESP32
    ledcAttachPin(PWM, PWM_CH);
}

loop() {
    // sp is the calculated PWM value
    pwmValue = sp;
    // ledcWrite for ESP32 writes the value in the channel
    ledcWrite(channel, pwmValue);

    // ESP8266 wirtes on the pin 
    analogWrite(pin, pwmValue);
}
```

## Bibliography

esp8266 library  
[https://github.com/esp8266/Arduino#using-platformio]()


pwm for ESP8266:  
[https://randomnerdtutorials.com/esp8266-pwm-arduino-ide/]()

[https://esp8266-arduino-spanish.readthedocs.io/es/latest/reference.html]()


pwm for ESP32:  
[https://randomnerdtutorials.com/esp32-pwm-arduino-ide/]()