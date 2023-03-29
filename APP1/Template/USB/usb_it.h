

#ifndef GD32C10X_IT_H
#define GD32C10X_IT_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "gd32c10x.h"

/* this function handles USB wakeup interrupt handler */
void USBFS_WKUP_IRQHandler(void);
/* this function handles USBFS IRQ Handler */
void USBFS_IRQHandler(void);
/* this function handles TIMER2 IRQ Handler */
void TIMER2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* GD32C10X_IT_H */
