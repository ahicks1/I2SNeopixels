/*
 * Demo code for driving digital RGB(W) LEDs using the ESP32's RMT peripheral
 *
 * Modifications Copyright (c) 2017 Martin F. Falatic
 *
 * Based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 */
/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <stdlib.h>

#include "driver/i2s.h"
#include "freertos/queue.h"


#include "nvs.h"
#include "nvs_flash.h"


#ifndef __cplusplus
#define nullptr  NULL
#endif



#define min(a, b)  ((a) < (b) ? (a) : (b))
#define max(a, b)  ((a) > (b) ? (a) : (b))
#define floor(a)   ((int)(a))
#define ceil(a)    ((int)((int)(a) < (a) ? (a+1) : (a)))

uint8_t inputTable[4] = {136,142,232,238};

static uint8_t outBuffer[1024] = {0};
static uint8_t offBuffer[1024] = {0};
uint8_t zeroBuffer[1] = {0};

uint32_t IRAM_ATTR millis()
{
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void color_to_out(uint8_t r,uint8_t g, uint8_t b, uint8_t idx) {
  int loc = idx*12;
  outBuffer[loc+0] = inputTable[g&3];
  outBuffer[loc+1] = inputTable[(g >> 2) & 3];
  outBuffer[loc+2] = inputTable[(g >> 4) & 3];
  outBuffer[loc+3] = inputTable[(g >> 6) & 3];
  outBuffer[loc+4] = inputTable[r & 3];
  outBuffer[loc+5] = inputTable[(r >> 2) & 3];
  outBuffer[loc+6] = inputTable[(r >> 4) & 3];
  outBuffer[loc+7] = inputTable[(r >> 6) & 3];
  outBuffer[loc+8] = inputTable[b & 3];
  outBuffer[loc+9] = inputTable[(b >> 2) & 3];
  outBuffer[loc+10] = inputTable[(b >> 4) & 3];
  outBuffer[loc+11] = inputTable[(b >> 6) & 3];
}

void app_main() {

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );


  #include "driver/i2s.h"
  #include "freertos/queue.h"

  static const int i2s_num = 0; // i2s port number

  /*static const i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX,
    .sample_rate = 2800000,
    .bits_per_sample = 16,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 16,
    .dma_buf_len = 64,
    .use_apll = false
  };*/
  static const i2s_config_t i2s_config = {
     .mode = I2S_MODE_MASTER | I2S_MODE_TX,
     .sample_rate = 44100,
     .bits_per_sample = 16,
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
     .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
     .intr_alloc_flags = 0, // default interrupt priority
     .dma_buf_count = 32,
     .dma_buf_len = 864,
     .use_apll = false
};

static const i2s_pin_config_t pin_config = {
  .bck_io_num = 26,
  .ws_io_num = 22,
  .data_out_num = 25,
  .data_in_num = I2S_PIN_NO_CHANGE
};


  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);   //install and start i2s driver

  i2s_set_pin(i2s_num, &pin_config);

  i2s_set_sample_rates(i2s_num, 93750); //set sample rates

  //color_to_out(255,0, 0, 0);
  for(int i = 0; i < 72; i++) {
    color_to_out(0, 50,0,i);
  }
  uint32_t written = 0;
  ESP_LOGI("LED","%hhX", outBuffer[0]);

  //i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
  //i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);

  //i2s_write(i2s_num, zeroBuffer, 1, &written, portMAX_DELAY);
  //i2s_write(i2s_num, outBuffer, 360, &written, portMAX_DELAY);
  printf("%hhX", outBuffer[0]);
  i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
  i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
  i2s_write(i2s_num, offBuffer, 50, &written, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(10));
  i2s_zero_dma_buffer(i2s_num);
  /*while(1) {
    for (int r  = 0; r < 255; r+=1 ) {
      for(int i = 0; i < 72; i++) {
        color_to_out(r, 0, 0,i);
      }

      i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
      i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
      i2s_write(i2s_num, offBuffer, 12, &written, portMAX_DELAY);
      ESP_LOGI("LED","bytes written %d",written);
      //i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(10));
      //i2s_zero_dma_buffer(i2s_num);
    }
    for (int rl = 255; rl > 0; rl-=1 ) {
      for(int il = 0; il < 72; il++) {
        color_to_out(rl, 0, 0,il);
      }

      i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
      i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
      i2s_write(i2s_num, offBuffer, 12, &written, portMAX_DELAY);
      ESP_LOGI("LED","bytes written %d",written);
      //i2s_write(i2s_num, outBuffer, 864, &written, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(10));
      //i2s_zero_dma_buffer(i2s_num);
    }*/


  //}
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }


  //i2s_driver_uninstall(i2s_num); //stop & destroy i2s driver

  //initialise_wifi();
  /* Temporarily pin task to core, due to FPU uncertainty */
  //xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 9216, NULL, 5, NULL, 1);
  //led_impl_init();
  //websocket_server_init();

  //vTaskDelete(NULL);
}
