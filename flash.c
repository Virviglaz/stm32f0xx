#include "stm32f0xx.h"
#include <errno.h>
#include "flash.h"

#define FLASH_PAGE_SIZE				0x400
#define CHECK_ALIGNMENT(x, y)			if ((x) % (y)) return EINVAL;

static void unlock(void)
{
	if ((FLASH->CR & FLASH_CR_LOCK)) {
		FLASH->KEYR = FLASH_FKEY1;
		FLASH->KEYR = FLASH_FKEY2;
	}
	
	while (FLASH->SR & FLASH_SR_BSY) { }
}

static void lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

/******************************** PUBLIC FUNCTIONS ****************************/
int flash_erase_page(uint32_t address)
{
	int res;

	CHECK_ALIGNMENT(address, FLASH_PAGE_SIZE);

	res = flash_blank(address, FLASH_PAGE_SIZE);
	if (!res) /* do not erase blank flash */
		return res;

	unlock();

	if (FLASH->SR & FLASH_SR_EOP) 
		FLASH->SR = FLASH_SR_EOP;	

	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = address;
	FLASH->CR |= FLASH_CR_STRT;
	while (!(FLASH->SR & FLASH_SR_EOP));
	FLASH->SR = FLASH_SR_EOP;
	FLASH->CR &= ~FLASH_CR_PER;

	lock();

	return 0;
}

int flash_erase(uint32_t address, uint32_t size)
{
	CHECK_ALIGNMENT(address, FLASH_PAGE_SIZE);
	CHECK_ALIGNMENT(size, FLASH_PAGE_SIZE);
		
	for (int a = address; a < address + size; a += FLASH_PAGE_SIZE)
		flash_erase_page(a);

	return 0;
}

int flash_write(uint32_t address, const uint32_t *src, uint32_t size)
{
	uint16_t *rd_ptr = (uint16_t *)src;
	uint16_t *wr_ptr = (uint16_t *)address;

	CHECK_ALIGNMENT(address, sizeof(address));
	CHECK_ALIGNMENT(size, sizeof(size));

	unlock();

	if (FLASH->SR & FLASH_SR_EOP) 	
		FLASH->SR = FLASH_SR_EOP;
	
	FLASH->CR |= FLASH_CR_PG;

	do {
		*wr_ptr++ = *rd_ptr++;
		while (FLASH->SR & FLASH_SR_BSY) { }
	} while (size -= sizeof(uint16_t));

	FLASH->CR &= ~FLASH_CR_PG;

	lock();

	return 0;
}

int flash_blank(uint32_t address, uint32_t size)
{
	CHECK_ALIGNMENT(address, sizeof(address));
	CHECK_ALIGNMENT(size, sizeof(size));

	for (uint32_t a = address; a < address + size; a += sizeof(uint32_t))
			if ( *(__IO uint32_t*)a != 0xFFFFFFFF) return EACCES;

	return 0;
}
