#include <stdint.h>
#include <avr/io.h>
#include "I2C_TEMP.h"

#define TC74ADDR 0x48

void setup() {
  Serial.begin(9600);
}

void loop() {
  i2c_init();
  i2c_start(TC74ADDR<<1);
  i2c_write(0x00);
  i2c_start((TC74ADDR<<1)+1);
  uint8_t temperature = i2c_read();
  i2c_stop();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  delay(1000);
}
