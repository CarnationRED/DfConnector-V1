
#include "gd32c10x_it.h"
#include "usbd_core.h"
#include "usb_conf.h"
#include "drv_usbd_int.h"
#include "custom_hid_core.h"

uint8_t send_buffer1[4] = {0x00, 0x01, 0x00, 0x00};
uint8_t send_buffer2[4] = {0x00, 0x01, 0x00, 0x00};
extern usb_core_driver  custom_hid;
extern uint32_t usbfs_prescaler;
extern void usb_timer_irq     (void);

/*!
    \brief      this function handles timer2 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}

/*!
    \brief      this function handles USBFS interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void  USBFS_IRQHandler (void)
{
    usbd_isr (&custom_hid);
}

/*!
    \brief      this function handles USBFS wakeup interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBFS_WKUP_IRQHandler(void)
{
    if (custom_hid.bp.low_power) {

        SystemInit();

        rcu_usb_clock_config(usbfs_prescaler);

        rcu_periph_clock_enable(RCU_USBFS);

         usb_clock_active(&custom_hid);
    }

    exti_interrupt_flag_clear(EXTI_18);
}

/*!
    \brief      this function handles EXTI0_IRQ Handler.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void  EXTI0_IRQHandler (void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_0)) {
        if (USBD_CONFIGURED == custom_hid.dev.cur_status) {
            send_buffer1[0] = 0x15U; 

            if (RESET == gd_eval_key_state_get(KEY_S1)) {
                if(send_buffer1[1]) {
                    send_buffer1[1] = 0x00U;
                } else {
                    send_buffer1[1] = 0x01U;
                }
            }

            custom_hid_report_send (&custom_hid, send_buffer1, 2U);
        }

        /* clear the EXTI line interrupt flag */
        exti_interrupt_flag_clear(EXTI_0);
    }
}


/*!
    \brief      this function handles EXTI10_15_IRQ Handler.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void  EXTI10_15_IRQHandler (void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_0)) {
        if (USBD_CONFIGURED == custom_hid.dev.cur_status) {
            send_buffer2[0] = 0x16U;

            if (RESET == gd_eval_key_state_get(KEY_S1)) {
                if(send_buffer2[1]) {
                    send_buffer2[1] = 0x00U;
                } else {
                    send_buffer2[1] = 0x01U;
                }
            }

            custom_hid_report_send (&custom_hid, send_buffer2, 2U);
        }

        /* clear the EXTI line interrupt flag */
        exti_interrupt_flag_clear(EXTI_0);
    }
}
