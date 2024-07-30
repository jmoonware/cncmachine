pin_size_t vOutPin = D0; // D0 = PWM5A = GPI26
uint8_t vOutSlice;

//pin_size_t motorPin = D2;  // D2 = PWM6A = GPIO28
//pin_size_t irqPin = D4; // program D9 = PWM2A = GPIO4 for IRQ
pin_size_t debugPin = D6; // jjust a pin to send signals for debugging


//pin_size_t limitOpenPin = D7; // limit switch, open lid
//pin_size_t limitClosePin = D8; // limit switch, close lid
//pin_size_t motorControlPin = D9; // relay for switching motor on/off 
//pin_size_t musicEnablePin = D10; // set high during open/speech/close states 

#define VOUT_TOP 13300 // for 133 MHz clock freq = 10 kHz pulse rate

#define i2c_addr 100 // hex 0x64

#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <elapsedMillis.h>
#include <Wire.h>

#define I2C_REG_LEN 10
#define I2C_BUFLEN 10
#define I2C_READ_ADDR_ERROR 0xFF

uint8_t i2c_data[I2C_REG_LEN];
uint8_t i2c_buf[I2C_BUFLEN];

// i2c_data[0] ==> blue led blink delay (in 10 ms increments)
#define I2C_BLUE_DELAY 0
#define I2C_PWM_VOUT_LEVEL_LOW 1
#define I2C_PWM_VOUT_LEVEL_HIGH 2

// put error codes here 
#define I2C_ERROR_REG 9

// I2C error codes
#define I2C_OK 0
#define I2C_ADDR_INVALID 1
#define I2C_TOO_MANY_BYTES 2


void i2c_receive(int numbytes)
{
        digitalWrite(PIN_LED_G, LOW);
        delay(5);
        digitalWrite(PIN_LED_G, HIGH);
        uint8_t i=0;
        while (Wire.available()) {
            if (i < I2C_BUFLEN) {
              i2c_buf[i]=Wire.read();
            }
            else {
              i2c_data[I2C_ERROR_REG]=I2C_TOO_MANY_BYTES;
            }
            i++;
        }
        uint8_t addr = 0;
        if (i==2) { // got address and data val
          addr = i2c_buf[0];
          if (addr < I2C_REG_LEN) {
            i2c_data[addr]=i2c_buf[1];
          }
          else {
            i2c_data[I2C_ERROR_REG]=I2C_ADDR_INVALID;
          }
        }
        else if (i==1) { // got address only - must be read
          addr = i2c_buf[0];
          if (addr < I2C_REG_LEN) {
            Wire.write(i2c_data[addr]);
          }
          else { // send garbage value
            Wire.write(I2C_READ_ADDR_ERROR);
            i2c_data[I2C_ERROR_REG]=I2C_ADDR_INVALID;
          }
        }
}

void setup() {

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);

  digitalWrite(PIN_LED_R, HIGH); // HIGH turns off
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, HIGH);

  gpio_set_function(vOutPin, GPIO_FUNC_PWM);
//  pinMode(irqPin, OUTPUT);
//  pinMode(debugPin, OUTPUT);
//  gpio_set_function(irqPin, GPIO_FUNC_PWM);
//  gpio_set_function(motorPin, GPIO_FUNC_PWM);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

// i2c pins
//  pinMode(motorControlPin, OUTPUT);
//  pinMode(limitOpenPin, INPUT);

  i2c_data[I2C_BLUE_DELAY] = 50; // 500 ms init
  i2c_data[I2C_ERROR_REG] = I2C_OK;

  // Start i2c as slave and set up handler
  Wire.begin(i2c_addr);
  Wire.onReceive(i2c_receive);

  vOutSlice = pwm_gpio_to_slice_num(vOutPin);
  pwm_config vOutConfig = pwm_get_default_config();
  pwm_config_set_wrap(&vOutConfig, VOUT_TOP); // number of clock cycles to update audio values
  pwm_init(vOutSlice, &vOutConfig, true);
  pwm_set_chan_level(vOutSlice, 0, 100); // just something to look at on scope
  i2c_data[I2C_PWM_VOUT_LEVEL_LOW]=100;
  i2c_data[I2C_PWM_VOUT_LEVEL_HIGH]=0;

//  irq_set_enabled(PWM_IRQ_WRAP, true);

}

void loop() {

        digitalWrite(PIN_LED_B, LOW);
        delay(10*i2c_data[I2C_BLUE_DELAY]);
        digitalWrite(PIN_LED_B, HIGH);
        delay(10*i2c_data[I2C_BLUE_DELAY]);
        pwm_set_chan_level(vOutSlice, 0, i2c_data[I2C_PWM_VOUT_LEVEL_LOW]+256*i2c_data[I2C_PWM_VOUT_LEVEL_HIGH]);
        
}
