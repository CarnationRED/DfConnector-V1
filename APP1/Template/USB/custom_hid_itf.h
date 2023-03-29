#include "custom_hid_core.h"

static void key_config(void);
static void led_config(void); 

static hid_fop_handler fop_handler;

/*!
    \brief      configure the keys
    \param[in]  none
    \param[out] none
    \retval     none
*/
void key_config(void)
{
    /* keys configuration */
    gd_eval_key_init(KEY_S1, KEY_MODE_EXTI);
    gd_eval_key_init(KEY_S2, KEY_MODE_EXTI);
}

/*!
    \brief      configure the LEDs
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_config(void)
{
    /* initialize LEDs */
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_init(LED4);
}
