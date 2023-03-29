#ifndef __MAIN__
#define __MAIN__
#include "delay.h"
#include "main.h"
#include "gd32c10x_fmc.h"
#define TOTAL_ROM_SIZE (128*1024U)
#define BT_ROM_SIZE (4*1024U)
#define APP_ROM_SIZE ((TOTAL_ROM_SIZE - BT_ROM_SIZE) / 2U)
#define ROM_BASE_ADDR 0x8000000U
#define APP_ROM1_ADDR (ROM_BASE_ADDR + BT_ROM_SIZE)
#define APP_ROM2_ADDR (APP_ROM1_ADDR + APP_ROM_SIZE)
#define BT_JUMP_ADDR (ROM_BASE_ADDR + BT_ROM_SIZE - 4U)
#define BT_JUMP_ADDR_PAGE (ROM_BASE_ADDR + BT_ROM_SIZE - 1024U)

void (*app_addr)();

u8 upgrade_to_rom1(void);

u8 rom_addr_valid(u32 addr)
{
		return ((*(uint32_t*)addr) & 0x2fff8000) == 0x20000000;
}

void init_led(void)
{
		rcu_periph_clock_enable(RCU_GPIOC);
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_13);//SIGNAL LED
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_14);//SIGNAL LED
}

void on_led(void)
{
		gpio_bit_set(GPIOC, GPIO_PIN_13);
		gpio_bit_set(GPIOC, GPIO_PIN_14);
}

void flash_led(void)
{
		FlagStatus on = gpio_output_bit_get(GPIOC, GPIO_PIN_13);
		if(on)
		{
				gpio_bit_reset(GPIOC, GPIO_PIN_13);
				gpio_bit_set(GPIOC, GPIO_PIN_14);
		}
		else {
				gpio_bit_set(GPIOC, GPIO_PIN_13);
				gpio_bit_reset(GPIOC, GPIO_PIN_14);
		}
}

void off_led(void)
{
		gpio_bit_reset(GPIOC, GPIO_PIN_13);
		gpio_bit_reset(GPIOC, GPIO_PIN_14);
}

void err_led(void)
{
		u8 i = 0;
		while(1)
		{
				delay_ms(250);
				gpio_bit_write(GPIOC, GPIO_PIN_13, i);
				gpio_bit_write(GPIOC, GPIO_PIN_14, i);
				if(i==0)i=1;
				else i=0;
		}
}

int main()
{
		uint32_t app_rom_addr;
		uint32_t app_run_addr;
		tim4_config();
		init_led();
//		err_led();
		on_led();
		
		app_rom_addr = *(__IO uint32_t*)BT_JUMP_ADDR;
		if(app_rom_addr != APP_ROM1_ADDR && app_rom_addr != APP_ROM2_ADDR)
		{
				u32 rom_addr = rom_addr_valid(APP_ROM1_ADDR) ? APP_ROM1_ADDR : APP_ROM2_ADDR;
			
				fmc_unlock();
				fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
				fmc_page_erase(BT_JUMP_ADDR_PAGE);
				fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
			
			
				app_rom_addr = 0;
				if(FMC_READY ==	fmc_word_program(BT_JUMP_ADDR, rom_addr))
						app_rom_addr = *(__IO uint32_t*)BT_JUMP_ADDR;
				
				
				fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
				fmc_lock();
				fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
		}
		
		if(app_rom_addr == APP_ROM2_ADDR && rom_addr_valid(APP_ROM2_ADDR) )
		{
				if(upgrade_to_rom1())
						app_rom_addr = APP_ROM1_ADDR;
				else app_rom_addr = 0x1;
		}
		
		app_run_addr = *(uint32_t*)app_rom_addr;
		
		if((app_run_addr & 0x2fff8000) == 0x20000000)
		{
				__disable_irq();
				//ram栈地址
				__set_MSP(app_run_addr);
//				nvic_vector_table_set(NVIC_VECTTAB_FLASH, app_rom_addr - ROM_BASE_ADDR);
				app_addr = (void(*)) (*(u32*)(app_rom_addr + 4));
				off_led();
				app_addr();
		}
		else
		{
			err_led();
		}
}

u8 upgrade_to_rom1(void)
{
		u32 from = APP_ROM2_ADDR;
		u32 to = APP_ROM1_ADDR;
		u16 pages = APP_ROM_SIZE / 1024;
		u16 i, j, res;
		
		fmc_unlock();
		fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
		
		for(i = 0; i < pages; i++)
		{
				fmc_page_erase(to);
				fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
					
				for(j = 0; j < 1024; j += 4)
				{
						fmc_word_program(to + j, *(u32*)(from + j));
				}
				to += 1024;
				from += 1024;
				flash_led();
		}
		fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
		fmc_page_erase(BT_JUMP_ADDR_PAGE);
		fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
		
		res = (FMC_READY ==	fmc_word_program(BT_JUMP_ADDR, APP_ROM1_ADDR));
		
		fmc_lock();
		fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
		
		return res;
}
#endif
