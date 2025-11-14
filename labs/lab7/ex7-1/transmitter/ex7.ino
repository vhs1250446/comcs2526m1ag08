#include "driver/gpio.h"
#include "driver/twai.h"

#define LED_BUILTIN 2

uint32_t value = 0;

void setup() {
    // put your setup code here, to run once:
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  delay(5000);
  Serial.begin(9600);
  //Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  //Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.printf("Driver installed\n");
  } else {
    Serial.printf("Failed to install driver\n");
  }
  //Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.printf("Driver started\n");
  } else {
    Serial.printf("Failed to start driver\n");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  //Configure message to transmit
  twai_message_t message;
  message.identifier = 0xAAAA;
  message.extd = 1;
  message.rtr = 0;
  message.data_length_code = 4;

  for (int i = 0; i < 4; i++) {
    message.data[i] = (value >> (i*8) & 0xFF);
  }
  Serial.printf("ID is 0x%lX ", message.identifier);
  //Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.printf("Message queued for transmission with %lu\n", value);
  } else {
    Serial.printf("Failed to queue message for transmission\n");
  }
  if (value == 1000) {
    value = 0;
  }
  value++;
  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
  delay(1000);
}
