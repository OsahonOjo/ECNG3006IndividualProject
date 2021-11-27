/* Osahon Ojo - 816005001
   ECNG 3006 Individual Project


 * I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.


 * TEST CODE BRIEF
 *
 * The finalized version of this task will traverse a 100-element array where each
 * element is of type uint16_t and find the indices of the maxima of the array elements.
 * Then the task will calculate the period between each maxima,
 * the average period between maxima,
 * and lastly, pulse rate in BPM.
 *
 * The task assumes that the values of the elements have an undulating pattern.
 *
 * For the purposes of this test, 20-element arrays will be used.
 *
 * For the peak-finding algorithm to detect an element as a maximum
 * it must be greater than the two elements before and after it.
 * Hence, the highest number of maxima in this test is 20 / 5 = 4;
 *
 * Test cases:
 * - TC1: 0 maxima
 * - TC2: 2 maxima
 * - TC3: 4 maxima.
 *
 * Pin assignment:
 * - GPIO2 is assigned as the data signal of i2c master port
 * - GPIO0 is assigned as the clock signal of i2c master port
 *
 * Connection:
 * - connect sda/scl of sensor with GPIO2/GPIO0
 *
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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


static xQueueHandle adc_readings_queue = NULL;
static xQueueHandle nn_interval_queue = NULL;
static xQueueHandle pulse_rate_queue = NULL;

static const char *TAG = "main";
static const char *TAG_UUT = "UUT";
static const char *TAG_TIME = "TIME_IN_MS";

static int N_ADC_READINGS = 20;
static int SAMPLING_PERIOD_IN_MS = 50;
static int N_MAXIMA_UPPER_LIMIT = 4;
static int QUEUE_SIZE_NN_INTERVAL = 30;
static int EFFECTIVE_PERIOD_IN_MS = 5000;

volatile uint16_t adc_result;
volatile uint16_t cmd_data;
volatile clock_t start;
volatile clock_t end;


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

static void calc_pulse_rate_task(void *arg)
{
    ESP_LOGI(TAG, "Now inside calc_pulse_rate_task\n");

    int ret = 0;

    volatile short test_arr[N_ADC_READINGS];

    test_arr[0] = 5;
    test_arr[1] = 5;
    test_arr[2] = 5;
    test_arr[3] = 5;
    test_arr[4] = 5;
    test_arr[5] = 5;
    test_arr[6] = 5;
    test_arr[7] = 5;
    test_arr[8] = 5;
    test_arr[9] = 5;
    test_arr[10] = 5;
    test_arr[11] = 5;
    test_arr[12] = 5;
    test_arr[13] = 5;
    test_arr[14] = 5;
    test_arr[15] = 5;
    test_arr[16] = 5;
    test_arr[17] = 5;
    test_arr[18] = 5;
    test_arr[19] = 5;

    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);
    xQueueSend(adc_readings_queue, test_arr, 0);

    ESP_LOGI(TAG_UUT, "TC1: 0 maxima\n");

    while (1)
    {
        volatile uint16_t adc_readings_arr[N_ADC_READINGS];
        volatile short maxima_indices_arr[N_MAXIMA_UPPER_LIMIT];
        volatile short nn_intervals_arr[N_MAXIMA_UPPER_LIMIT - 1];
        volatile int maxima_count = 0;
        volatile int pulse_rate = 0;

        for (int i = 0; i < N_MAXIMA_UPPER_LIMIT; i++)
        {
            if (i < N_MAXIMA_UPPER_LIMIT - 1)
                nn_intervals_arr[1] = -1;

            maxima_indices_arr[i] = -1;
        }

        start = clock();

        //Job A-1 take item off adc_readings_queue and store it in adc_readings_arr
        ESP_LOGI(TAG, "At Job A-1\n");

        if (xQueueReceive(adc_readings_queue, adc_readings_arr, 0))
        {
            ESP_LOGI(TAG, "At Job A-2\n");
            //Job A-2 find maxima indices, exclude the first and last two elements
            for (int i = 2; i < N_ADC_READINGS-3; i++)
            {
                if (adc_readings_arr[i] >= adc_readings_arr[i-1]
                    && adc_readings_arr[i] > adc_readings_arr[i-2]
                    && adc_readings_arr[i] > adc_readings_arr[i+1]
                    && adc_readings_arr[i] > adc_readings_arr[i+2])
                {
                     maxima_indices_arr[maxima_count] = i;
                     maxima_count++;
                }
            }

            ESP_LOGI(TAG, "At Job A-3\n");
            //Job A-3 calculate NN intervals, store them in nn_intervals_arr
            for (int i = 1; i < N_MAXIMA_UPPER_LIMIT; i++)
            {
                if (maxima_indices_arr[i] == -1)
                    break;

                nn_intervals_arr[i-1] = (maxima_indices_arr[i] - maxima_indices_arr[i-1]) * SAMPLING_PERIOD_IN_MS;
            }

            ESP_LOGI(TAG, "At Job A-4\n");
            //Job A-4 find pulse rate and send NN intervals to nn_interval_queue
            int nn_avg = 0, nn_count = 0;

            for (int i = 0; i < N_MAXIMA_UPPER_LIMIT - 1; i++)
            {
                if (nn_intervals_arr[i] == -1)
                    break;

                nn_avg += nn_intervals_arr[i];
                nn_count++;
            }

            nn_avg = nn_avg / nn_count;

            pulse_rate = 60000 / nn_avg;

            xQueueSend(pulse_rate_queue, &pulse_rate, 0);

            ESP_LOGI(TAG, "At Job A-5\n");
            //Job A-5 place NN intervals onto nn_interval_queue
            for (int i = 0; i < N_MAXIMA_UPPER_LIMIT - 1; i++)
            {
                if (nn_intervals_arr[i] != -1)
                    xQueueSend(nn_interval_queue, (nn_intervals_arr+1), 0);

            }

            ESP_LOGI(TAG_TIME, "Job A: %li ms\n", clock() - start);
            ESP_LOGI(TAG, "Pulse rate: %i BPM\n", pulse_rate);
            ESP_LOGI(TAG, "Average NN Interval: %i\n\n", nn_avg);

        }
        else
        {
            vTaskDelay(EFFECTIVE_PERIOD_IN_MS);
        }

    }

}

void app_main(void)
{
    ESP_LOGI(TAG, "Now inside app_main\n");

    //create queue to the adc reading, nn intervals and pulse rate queues
    adc_readings_queue = xQueueCreate(10, sizeof(uint16_t)*N_ADC_READINGS);//chng 10->2
    nn_interval_queue = xQueueCreate(QUEUE_SIZE_NN_INTERVAL, sizeof(uint8_t));
    pulse_rate_queue = xQueueCreate(10, sizeof(uint8_t));//chng 10->2

    //start i2c task
    xTaskCreate(calc_pulse_rate_task, "calc_pulse_rate_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "Now leaving app_main\n");
}
