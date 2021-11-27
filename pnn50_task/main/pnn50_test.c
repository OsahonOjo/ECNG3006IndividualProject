/* Osahon Ojo - 816005001
   ECNG 3006 Individual Project


 * I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.


 * TEST CODE BRIEF
 *
 * The finalized version of this task will take 30 or less NN intervals off a queue and
 * use them to calculate the pNN50 pulse rate variability as a percentage
 *
 * For the purposes of this test, only 20 items will be used.
 *
 * Test cases:
 * - TC1: 0 successive NN intervals with a difference over 50ms: 0%
 * - TC2: 6 pairs of successive NN intervals with a difference over 50ms: 30%
 * - TC3: 19 pairs of successive NN intervals with a difference over 50ms: 95%
 * - TC4: queue containing NN intervals is empty
 *
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"


#define I2C_EXAMPLE_MASTER_SCL_IO           0                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           2                /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

//ADS1115 slave address for ADDR pin connected to GND
#define ADS_ADDR			    0x48
//ADS1115 pointer register values for Conversion and Config registers
#define ADS_PTR_FOR_CONVERSION		    0x00
#define ADS_PTR_FOR_CONFIG		    0x01
//ADS1115 configuration word for initial configuration of the ADS1115
#define ADS_CONFIGWORD_INIT_CONFIG          0x4283
//ADS1115 configuration word to read from the ADS1115
#define ADS_CONFIGWORD_START_CONVERSION     0xC283


static xQueueHandle nn_interval_queue = NULL;
static xQueueHandle pnn50_queue = NULL;

static const char *TAG = "main";
static const char *TAG_UUT = "UUT";
static const char *TAG_TIME = "TIME_IN_MS";

static int QUEUE_SIZE = 2;
static int QUEUE_SIZE_NN_INTERVAL = 30;
static int EFFECTIVE_PERIOD_IN_MS = 5000;

volatile uint16_t adc_result;
volatile uint16_t cmd_data;
volatile clock_t start;
volatile clock_t end;

volatile uint8_t n_set_of_readings = 0;
static SemaphoreHandle_t mutex_n_set_of_readings;

/**
 * @brief i2c master initialization
 * this routine is appropriate because:
 * it sets the ESP as the master,
 * specifies the ESP pins that will act as SDA and SCL,
 * and disables internal pullups for those pins
 * since they have external pullups,
 * As such, when the i2c bus is idle, SDA and SCL are pulled high,
 * as they should be.
 */
static esp_err_t i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = 0;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief test code to write ADS1115
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_example_master_ADS1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief test code to read ADS1115
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_example_master_ADS1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_example_master_ADS1115_init(i2c_port_t i2c_num)
{
    ESP_ERROR_CHECK(i2c_example_master_init());
    cmd_data = ADS_CONFIGWORD_INIT_CONFIG;    // Config register value
    ESP_ERROR_CHECK(i2c_example_master_ADS1115_write(i2c_num, ADS_PTR_FOR_CONFIG, (uint8_t*) &cmd_data, 2));
    return ESP_OK;

    /* bit 15 (when writing): 0: DO NOT begin single conversion (when in power-down state)
       bit 14-12 (differential inputs): 100: single-sided input on channel 0: AINP = AIN0, AINN = GND
       bit 11-9 (full-scale input voltage range): 001: +/- 4.096V
       bit 8: 0: continuous conversion
       bit 7-5 (date rate): 100: 128SPS (default)
       bit 4 (COMP_MODE): 0: traditional comparator (default)
       bit 3 (COMP_POL): 0: active low (default)
       bit 2 (COMP_LAT): 0: nonlatching (default)
       bit 1-0 (COMP_QUE): 11: disable comparator (default)
       0b0100 0010 1000 0011 = 0x4283
     */
}

static void calc_pnn50_task(void *arg)
{
    ESP_LOGI(TAG, "Now inside calc_pnn50_task\n");

    int ret = 0;

    volatile uint16_t test_arr[20];

    test_arr[0] = 150;
    test_arr[1] = 50;
    test_arr[2] = 110;
    test_arr[3] = 50;
    test_arr[4] = 110;
    test_arr[5] = 50;
    test_arr[6] = 150;
    test_arr[7] = 50;
    test_arr[8] = 125;
    test_arr[9] = 50;
    test_arr[10] = 105;
    test_arr[11] = 50;
    test_arr[12] = 115;
    test_arr[13] = 50;
    test_arr[14] = 130;
    test_arr[15] = 50;
    test_arr[16] = 170;
    test_arr[17] = 50;
    test_arr[18] = 108;
    test_arr[19] = 50;

    while (1)
    {
        /*for (int i = 0; i < 20; i++)
        {
            xQueueSend(nn_interval_queue, (test_arr+i), 0);
        }*/

        n_set_of_readings = 7;

        volatile uint16_t prev_interval = 0;
        volatile uint16_t curr_interval = 0;
        volatile uint8_t nn50_count = 0;
        volatile uint8_t nn_count = 0;
        volatile uint16_t pnn50 = 0;

        if (xSemaphoreTake(mutex_n_set_of_readings, (TickType_t) 10))
        {
            if (n_set_of_readings >= 6)
            {
                start = clock();

                //Job A
                n_set_of_readings = 0;
                xSemaphoreGive(mutex_n_set_of_readings);

                if (xQueueReceive(nn_interval_queue, &prev_interval, 0))
                    nn_count = 1;

                while (xQueueReceive(nn_interval_queue, &curr_interval, 0))
                {
                    nn_count++;

                    if (curr_interval - prev_interval > 50
                        || curr_interval - prev_interval < -50)
                    {
                        nn50_count++;
                    }

                    prev_interval = curr_interval;
                }

                if (nn_count == 0)
                    pnn50 = 0;
                else
                    pnn50 = (nn50_count * 100) / nn_count;

                ESP_LOGI(TAG, "pNN50: %i %%\n", pnn50);
                ESP_LOGI(TAG_TIME, "Job A: %li ms\n", clock() - start);

            }
            else
            {
                xSemaphoreGive(mutex_n_set_of_readings);
                vTaskDelay(EFFECTIVE_PERIOD_IN_MS);
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Now inside app_main\n");

    //create queue to the adc reading, nn intervals and pulse rate queues
    nn_interval_queue = xQueueCreate(QUEUE_SIZE_NN_INTERVAL, sizeof(uint16_t));
    pnn50_queue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));

    //create mutex
    mutex_n_set_of_readings = xSemaphoreCreateMutex();

    //start i2c task
    xTaskCreate(calc_pnn50_task, "calc_pnn50_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "Now leaving app_main\n");
}
