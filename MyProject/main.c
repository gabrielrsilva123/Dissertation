#include <stdio.h>
#include <stdbool.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
/*#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif*/

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
#define TWI_INSTANCE_ID_2   1

#define I2C_SDA_2            23
#define I2C_SCL_2            24

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

  //MAX30101 Addresses 
  #define MAX30101_ADDRESS     0x57
  #define MAX30101_WR_DATA     (0xAE>>1)
  #define MAX30101_RD_DATA     (0xAF>>1)

  //MAX30101 Interrupt Registers
  #define MAX30101_INT_ST_1    0x00 
  #define MAX30101_INT_ST_2    0x01 
  #define MAX30101_INT_EN_1    0x02 
  #define MAX30101_INT_EN_2    0x03 

  //MAX30101 FiFO Registers
  #define MAX30101_FIFO_WR_PT  0x04
  #define MAX30101_OVF_CNT     0x05
  #define MAX30101_FIFO_RD_PT  0x06
  #define MAX30101_FIFO_DATA   0x07

  //MAX30101 Configuration Registers
  #define MAX30101_FIFO_CONF   0x08
  #define MAX30101_CONF_REG    0x09
  #define MAX30101_SPO2_CONF   0x0A

  //MAX30101 LEDx_PulseAmplitude
  #define LED1_PA              0x0C
  #define LED2_PA              0x0D
  #define LED3_PA              0x0E
  #define LED4_PA              0x0F

  #define MAX30101_ADD_LEN     0x01

  // -----------------
  //AS6221 Address
  #define AS6221_ADDRESS       0x46

  //AS6221 Index Registers
  #define AS6221_TVAL_REG      0x00
  #define AS6221_CONF_REG      0x01
  #define AS6221_TLOW_REG      0x02
  #define AS6221_THIGH_REG     0x03

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static const nrf_drv_twi_t m_twi2 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID_2);

static volatile bool m_xfer_done = false;

static uint8_t m_wr_ptr, m_rd_ptr, m_num_samples, m_data;
static uint8_t m_wr_ptr2, m_rd_ptr2, m_num_samples2, m_data2;

static uint32_t m_temp, m_red, m_ir, m_dc_avg;
static uint32_t m_temp2, m_red2, m_ir2, m_dc_avg2;


void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
	case NRF_DRV_TWI_EVT_DONE:
        m_xfer_done = true;
        break;
        
        default:
          break;
    }
}

/**
 * @brief TWI initialization.
 */

void i2c_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    const nrf_drv_twi_config_t twi_2_config = {
       .scl                = I2C_SCL_2,
       .sda                = I2C_SDA_2,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };


    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_init(&m_twi2, &twi_2_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    nrf_drv_twi_enable(&m_twi2);
} 

void MAX30101_scanner (void)
{
    ret_code_t err_code;
    uint8_t sample_data;

    NRF_LOG_INFO("MAX3101 scanner started.");
    NRF_LOG_FLUSH(); 

    err_code = nrf_drv_twi_rx(&m_twi, MAX30101_ADDRESS, &sample_data, sizeof(sample_data));
    APP_ERROR_CHECK(err_code);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO("MAX3101 device 1 detected at address 0x%x.", MAX30101_ADDRESS);
    }
    NRF_LOG_FLUSH();

    err_code = nrf_drv_twi_rx(&m_twi2, MAX30101_ADDRESS, &sample_data, sizeof(sample_data));
    APP_ERROR_CHECK(err_code);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO("MAX3101 device 2 detected at address 0x%x.", MAX30101_ADDRESS);
    }
    NRF_LOG_FLUSH();

}

void MAX30101_regWR (uint8_t reg_address, uint8_t value)
{
    ret_code_t err_code;
    uint8_t tx_buff[MAX30101_ADD_LEN+1];

    tx_buff[0] = reg_address;
    tx_buff[1] = value;


    //------------MAX30101 device 1----------------
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAX30101_ADDRESS, tx_buff, MAX30101_ADD_LEN+1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    //------------MAX30101 device 2----------------
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi2, MAX30101_ADDRESS, tx_buff, MAX30101_ADD_LEN+1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

void MAX30101_regRD (uint8_t reg_address, uint8_t * value_destination1, uint8_t * value_destination2)
{

    //------------MAX30101 device 1----------------
    ret_code_t err_code;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAX30101_ADDRESS, &reg_address, sizeof(reg_address), true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MAX30101_ADDRESS, value_destination1, sizeof(value_destination1));
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    //------------MAX30101 device 2----------------
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi2, MAX30101_ADDRESS, &reg_address, sizeof(reg_address), true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi2, MAX30101_ADDRESS, value_destination2, sizeof(value_destination2));
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

void MAX30101_FIFO_HR (uint32_t *pun_red_led, uint32_t *pun_red_led2)
{
    uint8_t status_1;
    uint8_t status_2;
    uint32_t temp_32 = 0;
    uint8_t temp_array[3]; // To read Heart Rate, it is necessary to read 3 bytes of the FIFO, because only a LED channel is used (RED)
    uint8_t temp_array2[3];

    *pun_red_led = 0;

    MAX30101_regRD(MAX30101_INT_ST_1, &status_1, &status_2);
    MAX30101_regRD(MAX30101_FIFO_DATA, temp_array, temp_array2);

    temp_32 = (temp_array[0] << 16)|(temp_array[1] << 8)|temp_array[2];
    *pun_red_led = temp_32;

    *pun_red_led &= 0x0003FFFF; 
    // The data output from the ADC is 18-bit long. 
    // This is done only to clean the signal and ensure the 32-bit word only has 18 bits of data.

    /*NRF_LOG_INFO("Primer temp_8: 0x%02x, segundo temp_8: 0x%02x, tercer temp_8: 0x%02x, temp_32 = 0x%08x", temp_array[0], temp_array[1], temp_array[2], temp_32);			
    NRF_LOG_FLUSH();*/

    temp_32 = 0;
    temp_32 = (temp_array2[0] << 16)|(temp_array2[1] << 8)|temp_array2[2];
    *pun_red_led2 = temp_32;

    *pun_red_led2 &= 0x0003FFFF; 
 
}

void MAX30101_FIFO_SpO2 (uint32_t *red_led, uint32_t *red_led2, uint32_t *ir_led, uint32_t *ir_led2)
{
    uint8_t status_1;
    uint8_t status_2;
    uint32_t red_read = 0;
    uint32_t red_read2 = 0;
    uint32_t ir_read = 0;
    uint32_t ir_read2 = 0;
    uint8_t temp_array[6]; // To read Heart Rate, it is necessary to read 3 bytes of the FIFO, because only a LED channel is used (RED)
    uint8_t temp_array2[6];

    *red_led = 0;
    *ir_led = 0;
    *red_led2 = 0;
    *ir_led2 = 0;

    MAX30101_regRD(MAX30101_INT_ST_1, &status_1, &status_2);
    MAX30101_regRD(MAX30101_FIFO_DATA, temp_array, temp_array2);

    //------------MAX30101 device 1----------------
    red_read = (temp_array[0] << 16)|(temp_array[1] << 8)|temp_array[2];
    *red_led = red_read;
    *red_led &= 0x0003FFFF; 
    // The data output from the ADC is 18-bit long. 
    // This is done only to clean the signal and ensure the 32-bit word only has 18 bits of data.

    ir_read = (temp_array[3] << 16)|(temp_array[4] << 8)|temp_array[5];
    *ir_led = ir_read;
    *ir_led &= 0x0003FFFF; 
    /*NRF_LOG_INFO("Primer temp_8: 0x%02x, segundo temp_8: 0x%02x, tercer temp_8: 0x%02x, temp_32 = 0x%08x", temp_array[0], temp_array[1], temp_array[2], temp_32);			
    NRF_LOG_FLUSH();*/


    //------------MAX30101 device 2----------------

    red_read2 = (temp_array2[0] << 16)|(temp_array2[1] << 8)|temp_array2[2];
    *red_led2 = red_read2;
    *red_led2 &= 0x0003FFFF; 

    ir_read2 = (temp_array[3] << 16)|(temp_array[4] << 8)|temp_array[5];
    *ir_led2 = ir_read2;
    *ir_led2 &= 0x0003FFFF; 

}

void MAX30101_init (void)
{
    //Initialization and Configuration of MAX30101
    MAX30101_regWR(MAX30101_INT_EN_1, 0xC0);
    MAX30101_regWR(MAX30101_INT_EN_2, 0x00);
    
    MAX30101_regWR(MAX30101_FIFO_WR_PT, 0x00);
    MAX30101_regWR(MAX30101_OVF_CNT, 0x00);
    MAX30101_regWR(MAX30101_FIFO_RD_PT, 0x00);

    MAX30101_regWR(MAX30101_FIFO_CONF, 0x5F);   // 4 samples averaged per FIFO sample - 0x5
    MAX30101_regWR(MAX30101_CONF_REG, 0x02);    //0x02 - HR; 0x03 - SpO2; 0x07 - Multi_LED
    MAX30101_regWR(MAX30101_SPO2_CONF, 0x27);
    MAX30101_regWR(LED1_PA, 0x3F);              // LED1 - RED
    MAX30101_regWR(LED2_PA, 0x00);              // LED2 - IR
    //MAX30101_regWR(LED3_PA, 0x3F);            // LED3 - GREEN
    //MAX30101_regWR(LED4_PA, 0x3F);            // LED4 - GREEN

    MAX30101_regRD(MAX30101_FIFO_WR_PT, &m_wr_ptr, &m_wr_ptr2);
    MAX30101_regRD(MAX30101_FIFO_RD_PT, &m_rd_ptr, &m_rd_ptr2);

    NRF_LOG_INFO("Initialization and Configuration of MAX30101 done");
    NRF_LOG_FLUSH();
}

void AS6221_scanner (void)
{
    ret_code_t err_code;
    uint8_t sample_data;

    NRF_LOG_INFO("AS6221 scanner started.");
    NRF_LOG_FLUSH();

    err_code = nrf_drv_twi_rx(&m_twi, AS6221_ADDRESS, &sample_data, sizeof(sample_data));
    APP_ERROR_CHECK(err_code);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO("AS6221 device detected at address 0x%x.", AS6221_ADDRESS);
    }
    NRF_LOG_FLUSH();

}

void AS6221_regWR (uint8_t reg_address, uint8_t most_sig_byte, uint8_t low_sig_byte)
{
    ret_code_t err_code;
    uint8_t tx_buff[3];

    tx_buff[0] = reg_address;
    tx_buff[1] = most_sig_byte;
    tx_buff[2] = low_sig_byte;

    m_xfer_done = false;

    err_code = nrf_drv_twi_tx(&m_twi, AS6221_ADDRESS, tx_buff, 3, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

uint8_t AS6221_regRD (uint8_t reg_address, uint8_t * most_sig_byte, uint8_t * low_sig_byte)
{
    ret_code_t err_code;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, AS6221_ADDRESS, &reg_address, sizeof(reg_address), true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, AS6221_ADDRESS, most_sig_byte, sizeof(most_sig_byte));
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, AS6221_ADDRESS, low_sig_byte, sizeof(low_sig_byte));
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

}

double AS6221_readT (double read_temp) 
{
    uint8_t temp_high_byte = 0x00;
    uint8_t temp_low_byte = 0x00;

    AS6221_regRD(AS6221_TVAL_REG, &temp_high_byte, &temp_low_byte); // Leitura da Temperatura

    read_temp = (temp_low_byte << 8)| temp_high_byte; 
    NRF_LOG_INFO("t_obtido = %d", read_temp);			
    NRF_LOG_FLUSH();

    read_temp = read_temp * 0.0078125;
    NRF_LOG_INFO("T = " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(read_temp));			
    NRF_LOG_FLUSH();

    return read_temp;
}

uint32_t DC_Offset (uint32_t arr[], uint32_t samples)
{
    uint32_t m_dc_offset = 0;
    for (uint32_t i = 0; i < samples; i++)
    {
        m_dc_offset = m_dc_offset + arr[i];
    }
    m_dc_offset = m_dc_offset/samples;
    return m_dc_offset;
}


uint32_t BPM(uint32_t arr[])
{
    uint32_t cnt = 0;
    uint32_t bpm_cnt = 0;

    uint32_t mean_test = DC_Offset(arr, 2500);

      for (uint32_t i = 3; i <= 2497; i++)
      {
      
        if (arr[i] > mean_test + 13500)
        {
          if ((arr[i] > arr[i-1]) && (arr[i] > arr[i-2]) && (arr[i] > arr[i-3]) && (arr[i] > arr[i+1]) && (arr[i] > arr[i+2]) && (arr[i] > arr[i+3]))
          {
            cnt++;
            //NRF_LOG_INFO("peak = %d", arr[i]);
            NRF_LOG_FLUSH();
          }
        }
       
         else
         {
            cnt = cnt;
         }
      }
    
    bpm_cnt = cnt * 4;
    //NRF_LOG_INFO("%d", bpm_cnt);
    //NRF_LOG_INFO("cnt  %d", cnt);
    //NRF_LOG_FLUSH();

    return  bpm_cnt;

    
}

double PerfIndex (uint32_t arr[], uint32_t dc_value)
{
    uint32_t per_index = 0;
    uint32_t ac_max = dc_value;
    //NRF_LOG_INFO("%d", dc_value);
    for (uint32_t i = 0; i < 2500; i++)
    {
        //NRF_LOG_INFO("%d", arr[i]);
        
        if (arr[i] < ac_max)
        {
            ac_max = arr[i];
        }
    }
    //NRF_LOG_INFO("%d", ac_max);
    NRF_LOG_FLUSH();
    per_index = (((double)dc_value - (double)ac_max) * 10000/(double)dc_value);
    return per_index;
}


int main(void)
{

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    i2c_init();
    MAX30101_scanner();
    MAX30101_init();
    //AS6221_scanner(); 

    double test_RD_Temp = 0x0000;
    uint8_t temp_high_byte = 0x00;
    uint8_t temp_low_byte = 0x00;
    uint32_t dc_average_red = 0;
    uint32_t dc_average_ir = 0;
    uint32_t samples = 0;
    uint32_t bpm_test = 0;
    uint32_t filtered_samples = 0;
    uint32_t filtered_samples2 = 0;

    uint32_t samples_one_min[2500];
    uint32_t samples_one_min2[2500];

    //AS6221_regWR(AS6221_CONF_REG, 0x42, 0xA0); // Configuração

    

    while (true)
    {

         MAX30101_regRD(MAX30101_FIFO_WR_PT, &m_wr_ptr, &m_wr_ptr2);
         MAX30101_regRD(MAX30101_FIFO_RD_PT, &m_rd_ptr, &m_rd_ptr);
         m_num_samples = m_wr_ptr - m_rd_ptr;
         m_num_samples2 = m_wr_ptr2 - m_rd_ptr2;
         uint32_t samples_dc [m_num_samples];
         
         //test_RD_Temp = AS6221_readT(test_RD_Temp);
         //nrf_delay_us(100000);

         // Based on the pseudo-code available on the MAX30101's datasheet
         for (uint8_t i = 0; i <  m_num_samples; i++)
         {
            //This is for HR! It is working!
            MAX30101_FIFO_HR(&m_temp, &m_temp2);
            //samples_dc[i] = m_temp;
            m_temp = m_temp - 225000; // DC Offset removal
            filtered_samples = filtered_samples - (0.0222 * filtered_samples - m_temp); // Low-pass filter
            NRF_LOG_INFO("%d", filtered_samples);
            
            //------------MAX30101 device 2----------------
            m_temp2 = m_temp2 - 225000; // DC Offset removal
            filtered_samples2 = filtered_samples2 - (0.0222 * filtered_samples2 - m_temp2); // Low-pass filter
            NRF_LOG_INFO("%d", filtered_samples2);
            //---------------------------------------------

            //This is for SpO2! It is Working!
            //MAX30101_FIFO_SpO2(&m_red, &m_red2, &m_ir, &m_ir2);
            //samples_dc[i] = m_red;
            //NRF_LOG_INFO("RED LED : %d", m_red);
            //NRF_LOG_INFO("IR LED : %d", m_ir);
            
            NRF_LOG_FLUSH();

            if (((samples + i) < 2500))
            {
              samples_one_min[samples + i] = filtered_samples;
              //samples_one_min[samples + i] = m_ir;
              //samples_one_min2[samples + i] = m_red;
            }
         }

         samples = samples + m_num_samples;

         //This is for HR! It is working!
         /*if (samples >= 2500)
         {
            bpm_test = BPM(samples_one_min);
            if (bpm_test > 40 && bpm_test < 140)
            {
                //NRF_LOG_INFO("BMP = %d", bpm_test);
                
            }
            else
            {
                //NRF_LOG_INFO("Measuring...");
            }
            //NRF_LOG_INFO("BMP = %d", bpm_test);
            NRF_LOG_FLUSH();
            samples = 0;
         }*/


         //This is for SpO2! It is working!
         /*if (samples >= 2500)
         {
           dc_average_ir = DC_Offset(samples_one_min, 2500);
           double pi_ir = PerfIndex(samples_one_min, dc_average_ir);

           dc_average_red = DC_Offset(samples_one_min2, 2500);
           double pi_red = PerfIndex(samples_one_min2, dc_average_red);

           double spo2_test = (1 - 0.51*(pi_red/pi_ir))*100;
           //NRF_LOG_INFO("IR PI = %d ", pi_ir);
           //NRF_LOG_INFO("RED PI = %d", pi_red);
           if (dc_average_red > 100000) // Only if there's a finger
           {
             //NRF_LOG_INFO("SpO2 = %d", spo2_test);
           }
           else
           {
              //NRF_LOG_INFO("No Finger");
           }

           NRF_LOG_FLUSH();
           samples = 0;
         }*/
    }
}

/** @} */
