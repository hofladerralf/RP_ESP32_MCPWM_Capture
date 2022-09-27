/**
 * for programming information see:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
 * https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/driver/driver/mcpwm.h
 * https://github.com/espressif/esp-idf/blob/526f682397a8cfb74698c601fd2c5b30e1433837/examples/peripherals/mcpwm/mcpwm_bldc_control/main/mcpwm_bldc_control_hall_sensor_example.c
 * 
 * https://github.com/JoaoLopesF/ESP32MotorControl/blob/master/ESP32MotorControl.cpp
 * 
 * APB-Clock-speed is 80MHz -> clockspeed for MCPWM
 *
 * 
 * Code tested and works safe from 10Hz to 25KHz
 * RP 27.09.2022
 * 
 */

#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"

#define GPIO_CAP0_IN   13                                       //Set GPIO 13 as  CAP0
#define CAP0_INT_EN BIT(27)                                     //Capture 0 interrupt bit

#define GPIO_CAP1_IN   27                                       //Set GPIO 27 as  CAP1
#define CAP1_INT_EN BIT(28)                                     //Capture 1 interrupt bit

//#define CAP2_INT_EN BIT(29)                                   //Capture 2 interrupt bit

//static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
static mcpwm_dev_t *MCPWM[1] = {&MCPWM0};
static uint32_t ICP0_val = 0;
static uint32_t ICP0_prev_val = 0;
static uint32_t ICP0_freq = 1;                                                        //init not with zero because divison through zero in first main...

static uint32_t ICP1_val = 0;
static uint32_t ICP1_prev_val = 0;
static uint32_t ICP1_freq = 1;                                                        //init not with zero because divison through zero in first main...


/**
 * ISR
 */
static void IRAM_ATTR ICPx_isr_handler(void *arg){
  uint32_t mcpwm_intr_status;
  mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val;                            //Read interrupt status
  //repeat the if-loop for CAP1_INT_EN or CAP2_INT_EN if needed
  if(mcpwm_intr_status & CAP0_INT_EN){                                            //Check for interrupt on rising edge on CAP0 signal
    ICP0_val = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);    //get capture signal counter value
    ICP0_freq = (ICP0_val - ICP0_prev_val);
    ICP0_prev_val = ICP0_val;
  }
  if(mcpwm_intr_status & CAP1_INT_EN){                                            //Check for interrupt on rising edge on CAP1 signal
    ICP1_val = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1);    //get capture signal counter value
    ICP1_freq = (ICP1_val - ICP1_prev_val);
    ICP1_prev_val = ICP1_val;
  }
  
  MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_IN);
  
  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);
  //mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);
  //enable interrupt, so each this a rising edge occurs interrupt is triggered
  MCPWM[MCPWM_UNIT_0] -> int_ena.val = (CAP0_INT_EN | CAP1_INT_EN);                                   //Enable interrupt on CAP0 or on all: (CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN)
  mcpwm_isr_register(MCPWM_UNIT_0, ICPx_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);        //Set ISR Handler
  
  
}

void loop() {
  float freq = 80000000.0f / ICP0_freq;       //calculate f in Hz works from 10Hz to 25KHz
  Serial.print("Freq0: ");
  Serial.println(freq);
  freq = 80000000.0f / ICP1_freq;
  Serial.print("Freq1: ");
  Serial.println(freq);
  Serial.println();

  delay(500);

}
