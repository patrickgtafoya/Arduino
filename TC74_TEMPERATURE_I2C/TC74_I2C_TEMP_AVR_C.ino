#include <stdint.h>
#include <avr/io.h>

#define TC74ADDR 0x48
#define SCL_CLOCK 100000
#define F_CPU 16000000UL
#define START 0x08
#define REP_START 0x10
#define MT_SLA_ACK 0x18
#define MR_SLA_ACK 0x40
#define MT_DATA_ACK 0x28
#define MR_DATA_NACK 0x58

void setup() {
  Serial.begin(9600);
}

void loop() {
  float Temperature = readTemp(TC74ADDR);
  Serial.print("Temperature: ");
  Serial.print(Temperature);
  Serial.print("C");
  Serial.print("\n");
  delay(1000);  
}

void i2c_init(){
  TWSR &= ~(1<<TWPS1)&~(1<<TWPS0);                              //Set Prescaler Value to 1
  TWBR = ((F_CPU/SCL_CLOCK)-16) / 2;                            //Set SCL clock to 100kHz  
}

uint8_t i2c_start(uint8_t address){
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);                       //Send Start Condition
  while(!(TWCR&(1<<TWINT)));                                    //Wait Until Transmission is Complete
  if((TWSR & 0xF8) != START && (TWSR & 0xF8) != REP_START){
    return 1;                                                   //Check for Start or Repeated Start condition transmitted
  }
    TWDR = (address);                                           //Send 7-bit Slave Address and R/W bit to TWDR
    TWCR = (1<<TWINT)|(1<<TWEN);                                //Clear TWINT to start transmission 
    while(!(TWCR&(1<<TWINT)));                                  //Wait for TWINT Flag
    if((TWSR&0xF8) != MT_SLA_ACK &&(TWSR&0xF8) != MR_SLA_ACK){ 
    return 1;                                                   //Check for either Transmit or Recieve ACK. Return 1 if not.
  }
  return 0;
}

uint8_t i2c_write(uint8_t data){
  TWDR = data;                                                  //Write address of register to read from
  TWCR = (1<<TWINT)|(1<<TWEN);                                  //Clear TWINT flag, begin transmission
  while(!(TWCR&(1<<TWINT)));                                    //Wait for TWINT flag
  if((TWSR & 0xF8)!=MT_DATA_ACK){
    return 1;                                                   //Check for data transmit ACK. If not, return 1.
  }
  return 0;
}

float i2c_read(void){
  TWCR = (1<<TWINT)|(1<<TWEN);                                  //Clear TWINT flag, begin transmission
  while(!(TWCR&(1<<TWINT)));                                    //Wait for TWINT flag
  if((TWSR & 0xF8) !=MR_DATA_NACK){
    return 1;                                                   //Check for data recieved NACK. If not, return 1
  }
  return TWDR;                                                  //Return data from RTR address
}

void i2c_stop(void){
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);                       //Send Stop Condition
}

float readTemp(uint8_t address){
  uint8_t SLA_W = (address<<1);
  uint8_t SLA_R = ((address<<1) + 1);
  i2c_init();
  i2c_start(SLA_W);
  i2c_write(0x00);
  i2c_start(SLA_R);
  float temp = i2c_read();
  i2c_stop();
  return (temp*(9.0/5.0) +32.0);
}

