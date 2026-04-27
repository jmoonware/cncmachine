//
// Code to control a RATTMMOTOR 500W CNC spindle
// The Rp2040 uses PWM to drive an opto-isolator, which turns the 10 V from the controller into 
// an adjustable 0-10V speed control voltage.
// An IR sensor module (KEAcvise IR obstacle sensor) detects the spindle rotation off a piece of white tape on the spindle surface.
// The RP2040 provides a slave-mode I2C interface to e.g. a Raspberry Pi 4 running Linux
//
pin_size_t vOutPin = D0; // D0 = PWM5A = GPI26, for PWM pulse output
pin_size_t ENCODER_SPEED_PIN = D7; // for edge detect input
uint8_t vOutSlice;

pin_size_t debugPin = D6; // just a pin to send signals for debugging

#define VOUT_TOP 13300 // for 133 MHz clock freq = 10 kHz pulse rate

#define i2c_addr 100 // hex 0x64

#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <Wire.h>

#define I2C_REG_LEN 32
#define I2C_BUFLEN 32

uint8_t i2c_data[I2C_REG_LEN];
uint8_t i2c_buf[I2C_BUFLEN];

// i2c_data[0] ==> blue led blink delay (in 10 ms increments)
#define I2C_BLUE_DELAY 0
#define I2C_PWM_VOUT_LEVEL_LOW 1
#define I2C_PWM_VOUT_LEVEL_HIGH 2
#define I2C_EDGE_TIME_LOW 3 // three bytes for microseconds between interrupts
#define I2C_EDGE_TIME_MID 4
#define I2C_EDGE_TIME_HIGH 5

// put error codes here 
#define I2C_ERROR_REG 9

// I2C error codes
#define I2C_ERR_OK 0
#define I2C_ERR_ADDR_INVALID 1
#define I2C_ERR_TOO_MANY_BYTES 2

unsigned long last_encoder_interrupt = micros();
unsigned long encoder_now = micros();
volatile unsigned long elapsed_micros = 0;

#define DEBOUNCE_US 500
volatile bool edgeCheckPending = false;

int64_t confirmEdgeLow(alarm_id_t id, void *user_data) {
  edgeCheckPending = false;

  // Confirm that the signal is still low after debounce interval
  if (digitalRead(ENCODER_SPEED_PIN) == LOW) {
    if (encoder_now > last_encoder_interrupt && last_encoder_interrupt != 0) {
      elapsed_micros = encoder_now - last_encoder_interrupt;
    }
    last_encoder_interrupt = encoder_now;
  }

  return 0;   // one-shot alarm
}


// isr for falling edge on D7 from encoder detector
// this sets an alarm for a little later to see if this is a "real" pulse
// shoud reject most spurious edge triggers
void encoderIsr() {

  if (edgeCheckPending) {
    return;   // ignore additional noise while validation is pending
  }

  encoder_now = micros();
  edgeCheckPending = true;
  add_alarm_in_us(DEBOUNCE_US, confirmEdgeLow, NULL, false);

}

// interrupt-safe-ish copy of elapsed_micros
void getEncoderTime() {
  
  noInterrupts();

  i2c_data[I2C_EDGE_TIME_LOW] = (uint8_t)(elapsed_micros&255); 
  i2c_data[I2C_EDGE_TIME_MID] = (uint8_t)((elapsed_micros>>8)&255);
  i2c_data[I2C_EDGE_TIME_HIGH] = (uint8_t)((elapsed_micros>>16)&255);

  interrupts();
}

// Current register/memory pointer
volatile uint16_t memAddr = 0;

// receive single byte data at single byte address
void i2c_receive(int numbytes)
{

  int nrec = 0;
  while (Wire.available()) {
    if (nrec < I2C_BUFLEN) {
      i2c_buf[nrec]=Wire.read();
    }
    else {
      i2c_data[I2C_ERROR_REG]=I2C_ERR_TOO_MANY_BYTES;
    }
    nrec++;
  }
  
  if (nrec >= 1) { // got a byte address 
    memAddr = i2c_buf[0];
    if (nrec >= 2) { // got a byte data val to write
      if (memAddr < I2C_REG_LEN) {
        i2c_data[memAddr]=i2c_buf[1];
      }
      else {
        i2c_data[I2C_ERROR_REG]=I2C_ERR_ADDR_INVALID;
      }
    }
  }
}

// write bytes back as requested by master
void i2c_request() {
  uint8_t sent = 0;

  while (sent < I2C_BUFLEN) {
    uint8_t out = 0xFF;
    // note memAddr set in receive above 
    if (memAddr < I2C_REG_LEN) {
      out = i2c_data[memAddr++];
    }
    Wire.write(out);
    sent++;
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
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

  i2c_data[I2C_BLUE_DELAY] = 7; //  update rate delay ms x2
  i2c_data[I2C_ERROR_REG] = I2C_ERR_OK;

  // Start i2c as slave and set up handlers
  Wire.begin(i2c_addr);
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_request);

  // set up PWM for motor control voltage adjustment
  vOutSlice = pwm_gpio_to_slice_num(vOutPin);
  pwm_config vOutConfig = pwm_get_default_config();
  pwm_config_set_wrap(&vOutConfig, VOUT_TOP); // number of clock cycles to update audio values
  pwm_init(vOutSlice, &vOutConfig, true);
  pwm_set_chan_level(vOutSlice, 0, 100); // just something to look at on scope
  i2c_data[I2C_PWM_VOUT_LEVEL_LOW]=100;
  i2c_data[I2C_PWM_VOUT_LEVEL_HIGH]=40; // should turn off effectively

  // encoder interrupt stuff
  pinMode(ENCODER_SPEED_PIN,INPUT_PULLUP);
  attachInterrupt(ENCODER_SPEED_PIN,encoderIsr,FALLING);
  // sets encoder edge irq to highest priority
  irq_set_priority(IO_IRQ_BANK0, 0);

}

uint16_t last_pwm_level = 0;
uint16_t current_pwm_level = 1;

void loop() {

        digitalWrite(PIN_LED_B, LOW);
        delay(10*i2c_data[I2C_BLUE_DELAY]);
        digitalWrite(PIN_LED_B, HIGH);
        delay(10*i2c_data[I2C_BLUE_DELAY]);

        // update I2C register
        getEncoderTime();

        current_pwm_level = i2c_data[I2C_PWM_VOUT_LEVEL_LOW]+256*i2c_data[I2C_PWM_VOUT_LEVEL_HIGH];
        if (current_pwm_level != last_pwm_level) {
          pwm_set_chan_level(vOutSlice, 0, current_pwm_level);
          last_pwm_level = current_pwm_level;
        }
}
