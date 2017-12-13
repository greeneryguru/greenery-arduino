/*
BSD 2-Clause License

Copyright (c) 2017, Jeffrey Leary
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <SimpleDHT.h>
#include <RCSwitch.h>

#define CMDBUFFER_SIZE 48

#define GETCMD 0
#define SETCMD 1
#define TXCMD 2

#define TEMPERATURE 0
#define HUMIDITY 1
#define SOILMOISTURE 2

#define DHT11 0
#define DHT22 1

#define ANALOG 0
#define DIGITAL 1

#define TXPULSEWIDTH 320
#define SWITCHED_POWER_PIN 13

// when analog sensors are connected to:
//  - a 5v pin: max range = 1023
//  - a 3.3v pin: max range = ? less than 5v, but not yet tested and determined. 
#define ANALOG_MAX_RANGE 1023


// GLOBAL VARS //
char cmdbuffer[CMDBUFFER_SIZE];
byte cmdindex = 0;


void setup() 
{
  pinMode(SWITCHED_POWER_PIN, OUTPUT);
  Serial.begin(9600);
}


void loop() 
{
  Serial.flush();
  if (Serial.available() > 0) {
    byte in;
    if (cmdindex >= CMDBUFFER_SIZE) {
      cmdindex = 0;
    }
    in = Serial.read();
    if (in == '\n') {
      cmdindex = 0;
      parse_command(cmdbuffer);
    } else {
      cmdbuffer[cmdindex++] = in;
      cmdbuffer[cmdindex] = '\0';
    }
  }
}


/*
 * parse command codes from a string.
 * after deciphering commands, appropriate functions are called.
 * 
 * params:
 *  a char buffer (\0 terminated, of course!)
 *  
 * returns:
 *  void
 */
void parse_command(char *buf) 
{
  bool failure = false;
  char pin[] = "00";
  
  if (int(buf[0] - '0') == GETCMD) {
    if (int(buf[1] - '0') == TEMPERATURE) {
      if (int(buf[2] - '0') == DHT22) {
        read_dht(atoi(&buf[3])); 
      }
    } else if (int(buf[1] - '0') == SOILMOISTURE) {
      if (int(buf[2] - '0') == ANALOG) {
        read_analog_moisture(atoi(&buf[3]));  
      }
    }
  } else if (int(buf[0] - '0') == TXCMD) {
    pin[0] = buf[1];
    pin[1] = buf[2];
    rftx(atoi(pin), strtoul(&buf[3], NULL, 0));
  } else {
      Serial.println("fail. unrecognized command char");
  }
  
  return;
}


/*
 *  read temp and humidity data from a DHT22 type sensor
 *  results are written to serial port in format:
 *  
 *    "t,<pin>,<value>\n"
 *    "h,<pin>,<value>\n"
 *    "ok\n"
 *  
 *  params:
 *    int pin-number
 *    
 *  returns:
 *    void
 */
void read_dht(int pin) 
{
  SimpleDHT22 dht22;
  float t = 0;
  float h = 0;
  int err = SimpleDHTErrSuccess;
  digitalWrite(SWITCHED_POWER_PIN, HIGH);
  delay(2100);
  
  if ((err = dht22.read2(pin, &t, &h, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("fail. err="); 
    Serial.println(err);  
    return;
  }

  digitalWrite(SWITCHED_POWER_PIN, LOW);
  
  Serial.print("t,");
  Serial.print((int)pin);
  Serial.print(",");
  Serial.println((float)t);

  Serial.print("h,");
  Serial.print((int)pin);
  Serial.print(",");
  Serial.println((float)h);
  
  Serial.println("ok");
  return;
}


/*
 *  read soil mositure from an analog pin
 *  results are written to serial port in format:
 *  
 *    "sm,<pin>,<value>\n"
 *    "ok\n"
 *  
 *  params:
 *    int pin-number
 *    
 *  returns:
 *    void
 */
void read_analog_moisture(int pin) 
{
  digitalWrite(SWITCHED_POWER_PIN, HIGH);
  delay(20);
  int n;
  n = analogRead(pin);
  digitalWrite(SWITCHED_POWER_PIN, LOW);
  
  // re-map the analog reading to a 0-100 scale
  n = map(n,ANALOG_MAX_RANGE,0,0,100);
  
  Serial.print("sm,");
  Serial.print((int)pin);
  Serial.print(",");
  Serial.println((int)n);
  Serial.println("ok");
  
  return;
}


/*
 *  send signals to rf transmit.
 *  generic settings. should work with most 433mhz RF outlets.
 *    "ok\n"
 *  
 *  params:
 *    int pin-number
 *    unsigned long transmission-code
 *    
 *  returns:
 *    void
 */
void rftx(int pin, unsigned long code) 
{ 
  RCSwitch sw = RCSwitch();
  sw.enableTransmit(pin);
  
  // Optional set pulse length.
  // sw.setPulseLength(TXPULSEWIDTH);
  
  // Optional set number of transmission repetitions.
  sw.setRepeatTransmit(5);
  
  digitalWrite(SWITCHED_POWER_PIN, HIGH);
  sw.send(code, 24);
  digitalWrite(SWITCHED_POWER_PIN, LOW);
  
  Serial.println("ok");
  return;
}

