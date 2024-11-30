#ifndef __STM32_FLASH_H__
#define __STM32_FLASH_H__

#include <stdint.h>

/**
 * @breif Flash erase single page.
 *
 * @param[in] address	Page address.
 *
 * @return 0 on success, EINVAL on error.
 */
int flash_erase_page(uint32_t address);

/**
 * @breif Flash erase address range.
 *
 * @param[in] address	Start address.
 * @param[in] size		Amount of bytes to erase.
 *
 * @return 0 on success, EINVAL on error.
 */
int flash_erase(uint32_t address, uint32_t size);

/**
 * @breif Write flash.
 *
 * @param[in] address	Start address.
 * @param[in] src		Source address buffer.
 * @param[in] size		Amount of bytes to write.
 *
 * @return 0 on success, EINVAL on error.
 */
int flash_write(uint32_t address, const uint32_t *src, uint32_t size);

/**
 * @breif Check flash in blank.
 *
 * @param[in] address	Start address.
 * @param[in] size		Amount of bytes to erase.
 *
 * @return 0 if flash is empty, EINVAL on error, EACCES if not empty.
 */
int flash_blank(uint32_t address, uint32_t size);

#endif /* __STM32_FLASH_H__ */
