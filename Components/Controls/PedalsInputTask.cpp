//#include "main_system.h"
//#include "stm32h5xx_hal_adc.h"
//
//extern ADC_HandleTypeDef hadc1;
//
//uint32_t adc_raw[4];
//uint32_t last_adc_update = 0;
//float adc_filtered[4] = {0};
//
//float accel_positive_voltage;
//float accel_negative_voltage;
//float brake_positive_voltage;
//float brake_negative_voltage;
//
//void pedals_adc_start(void) {
//    HAL_ADC_Start_DMA(&hadc1, adc_raw, 4);
//}
//
//void pedals_adc_filter(void) {
//    for (int i = 0; i < 4; i++) {
//        adc_filtered[i] = (0.1 * adc_raw[i]) + (0.9 * adc_filtered[i]);
//    }
//}
//
//void pedals(void) {
//    if ((HAL_GetTick() - last_adc_update) >= 50) { //20Hz = 50ms
//        last_adc_update = HAL_GetTick();
//        pedals_adc_filter();
//
//        accel_positive_voltage = (adc_filtered[0] / 4095.0) * 3.3; //accel positive
//        accel_negative_voltage = (adc_filtered[1] / 4095.0) * 3.3; //accel negative
//        brake_positive_voltage = (adc_filtered[2] / 4095.0) * 3.3; //brake positive
//        brake_negative_voltage = (adc_filtered[3] / 4095.0) * 3.3; //brake negative
//    }
//}
