#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

void app_main(void)
{
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 =
        {
         .unit_id = ADC_UNIT_1,
         .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config =
        {
         .bitwidth = ADC_BITWIDTH_9,
         .atten = ADC_ATTEN_DB_11,
        };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));

    /* Debug onboard led */
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));

    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_25, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_25, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_pullup_en(GPIO_NUM_25));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_26, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_26, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_pullup_en(GPIO_NUM_26));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_27, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_27, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_pullup_en(GPIO_NUM_27));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_pullup_en(GPIO_NUM_14));

    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timerCfg =
        {
         .group_id = 0,
         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
         .resolution_hz = 409600,
         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
         .period_ticks = 4096,
        };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timerCfg, &timer));

    mcpwm_oper_handle_t operator;
    mcpwm_operator_config_t operatorCfg = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operatorCfg, &operator));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator, timer));
    
    mcpwm_cmpr_handle_t compareTrackA, compareTrackB;
    mcpwm_comparator_config_t compareCfg = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &compareCfg, &compareTrackA));
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &compareCfg, &compareTrackB));

    mcpwm_gen_handle_t generateTrackA, generateTrackB;
    mcpwm_generator_config_t genCfgA =
        {
         .gen_gpio_num = GPIO_NUM_5,
        };
    ESP_ERROR_CHECK(mcpwm_new_generator(operator, &genCfgA, &generateTrackA));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event
                    (generateTrackA,
                     MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                  MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event
                    (generateTrackA,
                     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                    compareTrackA, MCPWM_GEN_ACTION_LOW)));
    mcpwm_generator_config_t genCfgB =
        {
         .gen_gpio_num = GPIO_NUM_4,
        };
    ESP_ERROR_CHECK(mcpwm_new_generator(operator, &genCfgB, &generateTrackB));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event
                    (generateTrackB,
                     MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                  MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event
                    (generateTrackB,
                     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                    compareTrackB, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_19, 0));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_18, 0));

    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_17, 0));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_16, 0));

    int prevTrackA = 0, prevTrackB = 0;
    while (1) {
        int valTrackA, valTrackB;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &valTrackA));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &valTrackB));

        int trackAForward = valTrackA && !gpio_get_level(GPIO_NUM_25);
        int trackABackward = valTrackA && !gpio_get_level(GPIO_NUM_26);
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_19, trackAForward ? 1 : 0));
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_18, trackABackward ? 1 : 0));
        int trackBForward = valTrackB && !gpio_get_level(GPIO_NUM_27);
        int trackBBackward = valTrackB && !gpio_get_level(GPIO_NUM_14);
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_17, trackBForward ? 1 : 0));
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_16, trackBBackward ? 1 : 0));

        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));
        if (valTrackA != prevTrackA) {
            if (valTrackA >> 4 != prevTrackA >> 4) {
                ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 1));
            }
            prevTrackA = valTrackA;
            ESP_LOGI("Power", "ADC%d track A Raw Data: %d", ADC_UNIT_1 + 1, valTrackA << 3);
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(compareTrackA, valTrackA << 3));
        }
        if (valTrackB != prevTrackB) {
            if (valTrackB >> 4 != prevTrackB >> 4) {
                ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 1));
            }
            prevTrackB = valTrackB;
            ESP_LOGI("Power", "ADC%d track B Raw Data: %d", ADC_UNIT_1 + 1, valTrackB << 3);
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(compareTrackB, valTrackB << 3));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    ESP_ERROR_CHECK(mcpwm_del_generator(generateTrackA));
    ESP_ERROR_CHECK(mcpwm_del_generator(generateTrackB));
    ESP_ERROR_CHECK(mcpwm_del_comparator(compareTrackA));
    ESP_ERROR_CHECK(mcpwm_del_comparator(compareTrackB));
    ESP_ERROR_CHECK(mcpwm_del_operator(operator));
    ESP_ERROR_CHECK(mcpwm_del_timer(timer));
}
