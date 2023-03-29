#ifndef __usb__
#define __usb__

#include "drv_usb_hw.h"
#include "custom_hid_core.h"
#include "custom_hid_itf.h"

usb_core_driver custom_hid;

void usb_init(void)
{
    usb_rcu_config();

    usb_timer_init();
	
		fop_handler.periph_config[0] = &key_config;
		fop_handler.periph_config[1] = &led_config;
    custom_hid_itfop_register(&custom_hid, &fop_handler);

    usbd_init(&custom_hid, USB_CORE_ENUM_FS, &custom_hid_desc, &usbd_custom_hid_cb);

    usb_intr_config();
}

#endif
