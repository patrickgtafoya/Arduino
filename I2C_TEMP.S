#include "avr/io.h"
/*  7:TWINT 6:TWEA 5:TWSTA 4:TWSTO 3:TWWC 2:TWEN 1:- 0:TWE
 *  TWBR = 0xB8
 *  TWCR = 0xBC
 *  TWSR = 0xB9
 *  TWDR = 0xBB
 */

  .file "I2C_TEMP.S"
  .text
  .global i2c_init
  .global i2c_start
  .global i2c_write
  .global i2c_read
  .global i2c_stop
  .global ERROR1

ERROR1:           ret

i2c_init:         ldi   r16, 0xF8         ;TWPS1 and TWPS0 set to 0
                  sts   TWSR, r16         ;Prescaler set to 0
                  ldi   r16, 0x48         ;(16000000/100000)-16 / 2 = 72 = 0x48
                  sts   TWBR, r16         ;Set SCL to 100kHz
                  ret

i2c_start:        ldi   r16, 0xA4         ;(1<<TWINT)|(1<<TWSTA)|(1<<TWEN) = 0xA4
                  sts   TWCR, r16         ;Send Start Condition
                  rcall wait              ;Wait for TWINT flag
                  lds   r16, TWSR         
                  andi  r16, 0xF8
                  ldi   r17, 0x08
                  cpse  r16, r17          ;Compare TWSR&0xF8 to start transmitted
                  rcall CHECK1            ;Compare TWSR&0xF8 to repeated start
                  sts   TWDR, r24         ;Send address (Parameters stored in r24)
                  ldi   r16, 0x84
                  sts   TWCR, r16         ;Clear TWINT flag     
                  rcall wait              ;Wait for TWINT flag
                  lds   r16, TWSR
                  andi  r16, 0xF8
                  ldi   r17, 0x18
                  cpse  r16, r17          ;Compare TWSR&0xF8 to MT_SLA_ACK
                  rcall CHECK2            ;Compare TWSR&0xF8 to MR_SLA_ACK
                  ret

i2c_write:        sts   TWDR, r24         ;Send data to TWDR(Parameters stored in r24)
                  ldi   r16, 0x84
                  sts   TWCR, r16         ;Clear TWINT flag   
                  rcall wait              ;Wait for TWINT flag
                  lds   r16, TWSR
                  andi  r16, 0xF8
                  cpi   r16, 0x28         ;Compare  TWSR&0xF8 to MT_DATA_ACK
                  brne  ERROR1
                  ret

i2c_read:         ldi   r16, 0x84
                  sts   TWCR, r16         ;Clear TWINT flag
                  rcall wait              ;Wait for TWINT flag
                  lds   r16, TWSR
                  andi  r16, 0xF8
                  cpi   r16, 0x58         ;Compare TWSR&0xF8 to MR_DATA_NACK
                  brne  ERROR1
                  lds   r24, TWDR         ;Load TWDR into r24(functions return to r24)
                  ret

i2c_stop:         ldi   r16, 0x94
                  sts   TWCR, r16
                  ret

wait:             lds   r16, TWCR
                  sbrs  r16, TWINT
                  rjmp  wait
                  ret

CHECK1:           ldi   r17, 0x10
                  cpse  r16, r17
                  rcall  ERROR1
                  ret

CHECK2:           ldi  r17, 0x40
                  cpse r16, r17
                  rcall ERROR1
                  ret
                  
