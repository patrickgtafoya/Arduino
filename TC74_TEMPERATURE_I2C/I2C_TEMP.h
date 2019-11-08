/*
*  Global register variables.
*/
#ifdef __ASSEMBLER__
/* Assembler-only stuff */
#else  /* !ASSEMBLER */
/* C-only stuff */
extern "C" void i2c_init(void);
extern "C" uint8_t i2c_start(uint8_t address);
extern "C" uint8_t i2c_write(uint8_t data);
extern "C" uint8_t i2c_read(void);
extern "C" void i2c_stop(void);
extern "C" uint8_t ERROR1(void);
#endif /* ASSEMBLER */
