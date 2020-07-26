#include "stdio.h"
#include "math.h"
#include <driver/adc.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"

#define SERIESRESISTOR 10000  // 10k Ohm resistor
#define THERMISTOR_TVALUE 25 // in Celcius unit
#define THERMISTOR_RVALUE 10000 // R at 25 Celcius
#define THERMISTOR_BCOEFF 3950 // params of NTC thermistor
#define VSOURCE     3.3 // 3.3 volt
#define VREF 1100 // 1100 millivolt

float get_resistance(float voltage, int resistor)
{
    float result = ((float)resistor * voltage) / (VSOURCE - voltage);

    return result;
}

float get_sampled_adc(int n_sample)
{
    float result = 0.0;
    for (int i = 0; i < n_sample; i++)
    {
        result += (float)(adc1_get_raw(ADC_CHANNEL_6)*1.0f);
    }

    return result / (n_sample*1.0f);
}

float calculate_temperature(float resistance)
{
    float t = 1.0 / ((float)THERMISTOR_TVALUE + 273.15);
    float b = 1.0 / ((float)THERMISTOR_BCOEFF);
    float l = (float)log((resistance/(float)THERMISTOR_RVALUE));

    float result = (1.0 / (t + (b * l))) - 273.15;

    return result;
}

void app_main()
{
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    esp_adc_cal_characteristics_t *adc_char = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, VREF, adc_char);

    if (type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("ADC eFuse Vref\r\n");
    }
    else if (type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("ADC two point\r\n");
    }
    else
    {
        printf("ADC default\r\n");
    }

    gpio_config_t io_cfg;
    io_cfg.intr_type = GPIO_INTR_DISABLE;
    io_cfg.mode = GPIO_MODE_OUTPUT;
    io_cfg.pin_bit_mask = GPIO_SEL_15;
    io_cfg.pull_down_en = 1;
    io_cfg.pull_up_en = 0;

    gpio_config(&io_cfg);

    while (1)
    {
        gpio_set_level(GPIO_NUM_15, 1);
        float sampled_adc = get_sampled_adc(64);
        uint32_t millivolt = esp_adc_cal_raw_to_voltage((uint32_t)sampled_adc, adc_char);
        float voltage = (float)millivolt / 1000.0f;
        float resistance = get_resistance(voltage, SERIESRESISTOR);
        float temperature = calculate_temperature(resistance);
        gpio_set_level(GPIO_NUM_15, 0);
    
        printf("adc_reading = %f\t V = %f\t R = %f\t T = %f *C\r\n", sampled_adc, voltage, resistance, temperature);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
