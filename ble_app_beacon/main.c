
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
//#include "boards.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "nrf_gpio.h"
#include "nrf.h"
#include "nrf_gpiote.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_delay.h"


#define LED_0 16

#define Pin_FromSoil 2
#define Pin_ToSoil 0
#define PIN_TEMP_ENABLE 6

#define DEVICE_NAME "SS"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define ADVERTISING_LED_PIN_NO           LED_0                             /**< Is on when device is advertising. */
#define INCREMENT_BUTTON                                 BUTTON_0

#define APP_CFG_NON_CONN_ADV_TIMEOUT     0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL     MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH           0x13                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH              0x11                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                  0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI                0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER           0xEFBE                           /**< Company identifier for Apple Inc. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                  0x01, 0x02                        /**< Major value used to identify Beacons. */ 
#define APP_MINOR_VALUE                  0x03, 0x04                        /**< Minor value used to identify Beacons. */ 

#define ADC_SAMPLING_INTERVAL                APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) /**< Sampling rate for the ADC */

static int32_t sample_num = 0;

static app_timer_id_t                        m_adc_sampling_timer_id;      /**< ADC timer */

// Hijack the UUID to send data, along with the MAJOR and MINOR values.
#define APP_BEACON_UUID                  0xDE, 0xAD, 0xBE, 0xEF, \
                                         0x45, 0x56, 0x67, 0x78, \
                                         0x89, 0x9a, 0xab, 0xbc
                                         //0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                        0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define MAJ_VAL_OFFSET_IN_BEACON_INFO    14                                /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                     0x10001100                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                     /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this 
                         // implementation. 
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the 
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value. 
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons. 
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons. 
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in 
                         // this implementation. 
};

static uint32_t TIME_DELAY = 0xFFFFFFFF;
static uint32_t TEMPERATURE = 0xFFFFFFFF;


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void gpio_init(void)
{
    //nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);

    // Init soil probes
    nrf_gpio_cfg_input(Pin_FromSoil, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(Pin_ToSoil);
		nrf_gpio_cfg_output(PIN_TEMP_ENABLE);

    // Init advertising LED
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
	
		// Enable temp sensor
		nrf_gpio_pin_set(PIN_TEMP_ENABLE);
}

void timers_init(void)
{
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Timer Mode
  NRF_TIMER0->PRESCALER = 0;                 // Prescaler 0 produces 16 MHz clock
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_16Bit;  // 16-bit mode

  NRF_TIMER0->TASKS_CLEAR = 1;               // clear the task first to be usable for later
  NRF_TIMER0->TASKS_START = 1;               // Start clock
    
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Timer Mode
  NRF_TIMER1->PRESCALER = 0;                 // Prescaler 0 produces 16 MHz clock
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;  // 16-bit mode
    
    NRF_TIMER1->TASKS_CLEAR = 1;               // clear the task first to be usable for later
}

void wait_to_zero(volatile NRF_TIMER_Type* p_timer, int timeToWait)
{
	//Waits for falling edge to completely zero out
	nrf_gpio_pin_clear(Pin_ToSoil);
	p_timer->TASKS_CLEAR = 1;
	p_timer->TASKS_START = 1;
	p_timer->CC[0] = timeToWait; 
	while (!p_timer->EVENTS_COMPARE[0])
	{}
	p_timer->EVENTS_COMPARE[0] = 0;
}

uint32_t find_time_delay(volatile NRF_TIMER_Type* p_timer, int numSamples)
{
    unsigned int difference[100];
    unsigned char i = 0;
    wait_to_zero(NRF_TIMER1, 2000);
    while (i < numSamples)
    {
        nrf_gpio_pin_set(Pin_ToSoil); //Send pulse
        p_timer->TASKS_CAPTURE[0] = 1; //Start timer
        while (!nrf_gpio_pin_read(Pin_FromSoil)) {} //Wait for return pulse
        p_timer->TASKS_CAPTURE[1] = 1; //Stop timer
        int currDifference = p_timer->CC[1] - p_timer->CC[0]; //Find time delay
            
        if (currDifference < 0xFFFF && currDifference > 0x0000) //Make sure delay is valid
        {
            difference[i++] = currDifference;
            wait_to_zero(NRF_TIMER1, 3 * currDifference);
        }
        else
            wait_to_zero(NRF_TIMER1, 2000);
    }
    
    //Average pulse duration
    uint32_t avgTime = 0;
    unsigned char step = 0;
    while (step < i)
    {
        avgTime += difference[step];
        step++;
    }
    avgTime /= step;
    
    return avgTime;
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t        err_code;
    ble_advdata_t   advdata;
    uint8_t         flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    //uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    //uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);
    uint16_t major_value = (TIME_DELAY & 0xFFFF0000) >> 16;
    uint16_t minor_value = (TIME_DELAY & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB(major_value);
    m_beacon_info[index++] = LSB(major_value);

    m_beacon_info[index++] = MSB(minor_value);
    m_beacon_info[index++] = LSB(minor_value);
		
		major_value = (TEMPERATURE & 0xFFFF0000) >> 16;
		minor_value = (TEMPERATURE & 0x0000FFFF);
		
		index = MAJ_VAL_OFFSET_IN_BEACON_INFO - 4;
		
		m_beacon_info[index++] = MSB(major_value);
    m_beacon_info[index++] = LSB(major_value);

    m_beacon_info[index++] = MSB(minor_value);
    m_beacon_info[index++] = LSB(minor_value);

    manuf_specific_data.data.p_data        = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size          = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
        
        // Try using full device name as "SS"
        // This has to be pretty short as each char is a UTF-8 char (2byte)
        advdata.name_type = BLE_ADVDATA_FULL_NAME;
        
        
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.p_manuf_specific_data   = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}

static void advertising_stop(void)
{
        uint32_t err_code;
    
        err_code = sd_ble_gap_adv_stop();
      APP_ERROR_CHECK(err_code);

        //
      // WTF how to disable pin
      //
    //nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, false);

    // Use the internal RC circuit with 16MHz external clock for calibration
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Set device name
    ble_gap_conn_sec_mode_t my;
    my.sm = 1;
    my.lv = 1;
    err_code = sd_ble_gap_device_name_set(&my, (uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}


// Disable the soft device and the BLE stack.
static void ble_stack_stop(void)
{
    uint32_t err_code;
    
    err_code = softdevice_handler_sd_disable();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
        // Sleep until interrupt recv
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

// ADC timer handler to start ADC sampling
static void adc_sampling_timeout_handler(void)
{
    uint32_t p_is_running = 0;
        
    sd_clock_hfclk_request();
    while(! p_is_running) {                             //wait for the hfclk to be available
        sd_clock_hfclk_is_running((&p_is_running));
    }               
    NRF_ADC->TASKS_START = 1;                           //Start ADC sampling
}

static void adc_init(void)
{   
	
		//nrf_delay_us(1000000);
	
    /* Enable interrupt on ADC sample ready event*/     
    NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;   
    sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);  
    sd_nvic_EnableIRQ(ADC_IRQn);
    
    NRF_ADC->CONFIG = (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /* Bits 17..16 : ADC external reference pin selection. */
                                    | (ADC_CONFIG_PSEL_AnalogInput2 << ADC_CONFIG_PSEL_Pos)                 /*!< Use analog input 2 as analog input. */
                                    | (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)                          /*!< Use internal 1.2V bandgap voltage as reference for conversion. */
                                    | (ADC_CONFIG_INPSEL_AnalogInputNoPrescaling << ADC_CONFIG_INPSEL_Pos) /*!< Analog input specified by PSEL with no prescaling used as input for the conversion. */
                                    | (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);                                  /*!< 8bit ADC resolution. */ 
    
    /* Enable ADC*/
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
}

/* Interrupt handler for ADC data ready event */
void ADC_IRQHandler(void)
{
    /* Clear dataready event */
  NRF_ADC->EVENTS_END = 0;  
	
	if (!sample_num) {
		NRF_ADC->TASKS_START = 1;
		sample_num++;
		return;
	}

  /* Write ADC result to tempurature */
	TEMPERATURE = NRF_ADC->RESULT;
	
	sd_nvic_DisableIRQ(ADC_IRQn);
	nrf_gpio_pin_clear(PIN_TEMP_ENABLE);
    
    //Use the STOP task to save current. Workaround for PAN_028 rev1.5 anomaly 1.
  NRF_ADC->TASKS_STOP = 1;
    
    //Release the external crystal
    sd_clock_hfclk_release();
	
		NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;
	
	// Start advertisment.
		advertising_init();
    advertising_start();

    // Only keep LED on for 200000 us
    //nrf_delay_us(200000);
    //nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
		
		//nrf_delay_us(1000000);
		//NVIC_SystemReset();
}  

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize gpio pins
    gpio_init();

    timers_init();

    // Find sensor data
		uint32_t lastSample = 0;
		uint32_t currSample = 0;
		const uint32_t epsilon = 15;
		//do {
				lastSample = currSample;
				currSample = find_time_delay(NRF_TIMER0, 100);
		//} while (lastSample != currSample);
		TIME_DELAY = currSample;

    // Init BLE
    ble_stack_init();
		
		// Read temperature
		adc_init();
		adc_sampling_timeout_handler();
		

    // Enter main loop.
    for (;;)
    {
        power_manage();
            
            // For doing readings in a loop use the following steps:
            // Disable advertisement using advertising_stop()
            // Disable the BLE stack by calling ble_stack_stop();
            // Re-initalize timers to what's needed for sensing
            // Perform sensing measurement
            // Re-initialize the BLE stack using ble_stack_init();
            // Re-initialize the advertisment data using advertising_init();
            // Start advertising using advertising_start();
    }
}

/**
 * @}
 */
