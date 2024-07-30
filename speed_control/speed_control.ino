pin_size_t vOutPin = D0; // D0 = PWM5A = GPI26
//pin_size_t motorPin = D2;  // D2 = PWM6A = GPIO28
//pin_size_t irqPin = D4; // program D9 = PWM2A = GPIO4 for IRQ
pin_size_t debugPin = D6; // jjust a pin to send signals for debugging


//pin_size_t limitOpenPin = D7; // limit switch, open lid
//pin_size_t limitClosePin = D8; // limit switch, close lid
//pin_size_t motorControlPin = D9; // relay for switching motor on/off 
//pin_size_t musicEnablePin = D10; // set high during open/speech/close states 

#define i2c_addr 100 // hex 0x64

#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <elapsedMillis.h>
#include <Wire.h>

#define I2C_REG_LEN 10
#define I2C_BUFLEN 10

uint8_t i2c_data[I2C_REG_LEN];
uint8_t i2c_buf[I2C_BUFLEN];

#define I2C_BLUE_DELAY 0
// i2c_data[0] ==> blue led blink delay (in 10 ms increments)

void i2c_receive(int numbytes)
{
        digitalWrite(PIN_LED_G, LOW);
        delay(50);
        digitalWrite(PIN_LED_G, HIGH);
        uint8_t i=0;
        while (Wire.available()) {
            if (i < I2C_BUFLEN) {
              i2c_buf[i]=Wire.read();
            }
            i++;
        }
        uint8_t addr = 0;
        if (i==2) { // got address and data val
          addr = i2c_buf[0];
          if (addr < I2C_REG_LEN) {
            i2c_data[addr]=i2c_buf[1];
          }
        }
        else if (i==1) { // got address only - must be read
          addr = i2c_buf[0];
          if (addr < I2C_REG_LEN) {
            Wire.write(i2c_data[addr]);
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
  // Start i2c as slave and set up handler
  Wire.begin(i2c_addr);
  Wire.onReceive(i2c_receive);


}

void loop() {

        digitalWrite(PIN_LED_B, LOW);
        delay(10*i2c_data[I2C_BLUE_DELAY]);
        digitalWrite(PIN_LED_B, HIGH);
        delay(10*i2c_data[I2C_BLUE_DELAY]);
}
