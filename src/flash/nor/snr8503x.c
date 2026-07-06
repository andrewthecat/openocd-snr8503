// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <inttypes.h>

#include <helper/binarybuffer.h>
#include <jtag/jtag.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/register.h>

#include "imp.h"

#define SNR8503X_FLASH_SIZE		0x8000
#define SNR8503X_FLASH_SECTOR_SIZE	0x200

#define SNR8503X_SYS_CLK_CFG		0x40000080
#define SNR8503X_SYS_WR_PROTECT		0x400000a8
#define SNR8503X_SYS_FLSE		0x400000d0
#define SNR8503X_SYS_FLSP		0x400000d4

#define SNR8503X_FLASH_CFG		0x00010000
#define SNR8503X_FLASH_ADDR		0x00010004
#define SNR8503X_FLASH_WDATA		0x00010008
#define SNR8503X_FLASH_ERASE		0x00010010
#define SNR8503X_FLASH_PROTECT		0x00010014
#define SNR8503X_FLASH_READY		0x00010018

#define SNR8503X_SYS_UNLOCK		0x00007a83
#define SNR8503X_FLASH_ERASE_ENABLE	0x00008fca
#define SNR8503X_FLASH_PROGRAM_ENABLE	0x00008f35
#define SNR8503X_FLASH_ERASE_KEY	0x7654dcba

#define SNR8503X_CLK_FLASH_BITS		0x000001ff
#define SNR8503X_CFG_CHIP_ERASE		0x80008000
#define SNR8503X_CFG_SECTOR_ERASE	0x80000000
#define SNR8503X_CFG_PROGRAM		0x08000000
#define SNR8503X_CFG_NVR		0x00000800
#define SNR8503X_READY			1

#define SNR8503X_TIMEOUT_MS		10000
#define SNR8503X_MASS_ERASE_TIMEOUT_MS	60000
#define SNR8503X_RAM_WRITE_RETRIES	5
#define SNR8503X_ALGO_STACK_SIZE	128
#define SNR8503X_ALGO_TRAMPOLINE_SIZE	4
#define SNR8503X_WRITE_CHUNK_SIZE	256

#define SNR8503X_ALGO_ERASE_CHIP_OFFSET	0x08
#define SNR8503X_ALGO_PROGRAM_PAGE_OFFSET	0x4e

static const uint8_t snr8503x_flash_algo[] = {
	0x00, 0x20, 0x70, 0x47, 0x00, 0x20, 0x70, 0x47,
	0x26, 0x48, 0x25, 0x49, 0x81, 0x62, 0x01, 0x68,
	0x25, 0x4a, 0x11, 0x43, 0x01, 0x60, 0x23, 0x49,
	0x24, 0x48, 0x40, 0x31, 0x08, 0x61, 0x88, 0x13,
	0x01, 0x68, 0x23, 0x4a, 0x11, 0x43, 0x01, 0x60,
	0x22, 0x49, 0x01, 0x61, 0x81, 0x69, 0x01, 0x29,
	0xfc, 0xd1, 0x41, 0x69, 0x01, 0x29, 0x06, 0xd0,
	0x01, 0x68, 0x1d, 0x4a, 0xd2, 0x43, 0x11, 0x40,
	0x01, 0x60, 0x00, 0x20, 0x70, 0x47, 0x01, 0x20,
	0x70, 0x47, 0x00, 0x20, 0x70, 0x47, 0x30, 0xb5,
	0x14, 0x4b, 0x13, 0x4c, 0x9c, 0x62, 0x1c, 0x68,
	0x13, 0x4d, 0x2c, 0x43, 0x1c, 0x60, 0x13, 0x4b,
	0x10, 0x4c, 0x95, 0x3b, 0x40, 0x34, 0x63, 0x61,
	0xa3, 0x13, 0x1c, 0x68, 0xdd, 0x02, 0x2c, 0x43,
	0x1c, 0x60, 0xc9, 0x1c, 0x89, 0x08, 0x89, 0x00,
	0x0a, 0xe0, 0x84, 0x08, 0xa4, 0x00, 0x5c, 0x60,
	0x14, 0x68, 0x9c, 0x60, 0x9c, 0x69, 0x01, 0x2c,
	0xfc, 0xd1, 0x12, 0x1d, 0x09, 0x1f, 0x00, 0x1d,
	0x00, 0x29, 0xf2, 0xd1, 0x18, 0x68, 0xa8, 0x43,
	0x18, 0x60, 0x00, 0x20, 0x30, 0xbd, 0x00, 0x00,
	0x83, 0x7a, 0x00, 0x00, 0x80, 0x00, 0x00, 0x40,
	0xff, 0x01, 0x00, 0x00, 0xca, 0x8f, 0x00, 0x00,
	0x00, 0x80, 0x00, 0x80, 0xba, 0xdc, 0x54, 0x76,
};

static target_addr_t snr8503x_algo_trampoline_addr(const struct working_area *algo_wa)
{
	return algo_wa->address + sizeof(snr8503x_flash_algo);
}

static target_addr_t snr8503x_algo_done_loop_addr(const struct working_area *algo_wa)
{
	return snr8503x_algo_trampoline_addr(algo_wa);
}

static int snr8503x_mass_erase(struct flash_bank *bank);

static int snr8503x_wait_ready(struct target *target, unsigned int timeout_ms)
{
	uint32_t ready;
	unsigned int read_errors = 0;

	for (unsigned int i = 0; i < timeout_ms; i++) {
		int retval = target_read_u32(target, SNR8503X_FLASH_READY, &ready);
		if (retval != ERROR_OK) {
			/*
			 * During erase the MEM-AP often returns transient WAITs on this
			 * device. Treat those as "not ready yet" and keep polling.
			 */
			read_errors++;
			alive_sleep(1);
			continue;
		}

		if (ready == SNR8503X_READY)
			return ERROR_OK;

		alive_sleep(1);
	}

	if (read_errors)
		LOG_DEBUG("SNR8503x wait_ready saw %u transient read errors", read_errors);

	LOG_ERROR("SNR8503x flash operation timed out");
	return ERROR_FAIL;
}

static int snr8503x_read_u32_retry(struct target *target, target_addr_t address,
		uint32_t *value, unsigned int timeout_ms)
{
	int last_retval = ERROR_FAIL;

	for (unsigned int i = 0; i < timeout_ms; i++) {
		last_retval = target_read_u32(target, address, value);
		if (last_retval == ERROR_OK)
			return ERROR_OK;

		alive_sleep(1);
	}

	return last_retval;
}

static int snr8503x_prepare(struct target *target)
{
	uint32_t value;
	int retval;

	retval = target_write_u32(target, SNR8503X_SYS_WR_PROTECT, SNR8503X_SYS_UNLOCK);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, SNR8503X_SYS_CLK_CFG, &value);
	if (retval != ERROR_OK)
		return retval;

	return target_write_u32(target, SNR8503X_SYS_CLK_CFG, value | SNR8503X_CLK_FLASH_BITS);
}

static int snr8503x_cleanup(struct target *target)
{
	int retval = target_write_u32(target, SNR8503X_SYS_FLSE, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, SNR8503X_SYS_FLSP, 0);
	if (retval != ERROR_OK)
		return retval;

	uint32_t value;
	retval = target_read_u32(target, SNR8503X_FLASH_CFG, &value);
	if (retval != ERROR_OK)
		return retval;

	value &= ~(SNR8503X_CFG_CHIP_ERASE | SNR8503X_CFG_SECTOR_ERASE |
			SNR8503X_CFG_PROGRAM | SNR8503X_CFG_NVR);
	return target_write_u32(target, SNR8503X_FLASH_CFG, value);
}

static int snr8503x_set_reg_u32(struct target *target, const char *name, uint32_t value)
{
	struct reg *reg = register_get_by_name(target->reg_cache, name, true);
	uint8_t buf[4];

	if (!reg || !reg->exist) {
		LOG_ERROR("SNR8503x missing register %s", name);
		return ERROR_FAIL;
	}

	buf_set_u32(buf, 0, 32, value);
	return reg->type->set(reg, buf);
}

static int snr8503x_write_buffer_retry(struct target *target, target_addr_t address,
		uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_FAIL;

	for (unsigned int attempt = 0; attempt < SNR8503X_RAM_WRITE_RETRIES; attempt++) {
		retval = target_write_buffer(target, address, count, buffer);
		if (retval == ERROR_OK)
			return ERROR_OK;

		LOG_DEBUG("SNR8503x RAM write retry %u/%u at 0x%08" TARGET_PRIxADDR,
				attempt + 1, SNR8503X_RAM_WRITE_RETRIES, address);
		alive_sleep(1);
	}

	return retval;
}

static int snr8503x_get_reg_u32(struct target *target, const char *name, uint32_t *value)
{
	struct reg *reg = register_get_by_name(target->reg_cache, name, true);

	if (!reg || !reg->exist) {
		LOG_ERROR("SNR8503x missing register %s", name);
		return ERROR_FAIL;
	}

	*value = buf_get_u32(reg->value, 0, reg->size);
	return ERROR_OK;
}

struct snr8503x_saved_context {
	uint32_t pmsk_bpri_fltmsk_ctrl;
	uint32_t xpsr;
	uint32_t sp;
	uint32_t lr;
	uint32_t pc;
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
};

static int snr8503x_run_algo(struct target *target, target_addr_t entry,
		target_addr_t trampoline, target_addr_t done_loop,
		target_addr_t stack_top, target_addr_t data_addr, uint32_t flash_addr,
		uint32_t count, uint32_t *result)
{
	int retval;

	retval = snr8503x_set_reg_u32(target, "pmsk_bpri_fltmsk_ctrl", 1);
	if (retval != ERROR_OK)
		return retval;
	retval = snr8503x_set_reg_u32(target, "xpsr", 0x01000000);
	if (retval != ERROR_OK)
		return retval;
	retval = snr8503x_set_reg_u32(target, "pc", entry | 1U);
	if (retval != ERROR_OK)
		return retval;
	retval = snr8503x_set_reg_u32(target, "lr", trampoline | 1U);
	if (retval != ERROR_OK)
		return retval;
	retval = snr8503x_set_reg_u32(target, "sp", stack_top);
	if (retval != ERROR_OK)
		return retval;
	retval = snr8503x_set_reg_u32(target, "r0", flash_addr);
	if (retval != ERROR_OK)
		return retval;
	retval = snr8503x_set_reg_u32(target, "r1", count);
	if (retval != ERROR_OK)
		return retval;
	retval = snr8503x_set_reg_u32(target, "r2", data_addr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_resume(target, false, entry, true, true);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * This target/J-Link combination is prone to SWD WAITs if OpenOCD starts
	 * polling immediately after resuming RAM code during flash programming.
	 * Give the helper a short head start before waiting for the BKPT halt.
	 */
	alive_sleep(1);

	retval = target_wait_state(target, TARGET_HALTED, SNR8503X_TIMEOUT_MS);
	if (retval != ERROR_OK || target->state != TARGET_HALTED) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
		retval = target_wait_state(target, TARGET_HALTED, 500);
		if (retval != ERROR_OK)
			return retval;
		return ERROR_TARGET_TIMEOUT;
	}

	uint32_t pc;
	retval = snr8503x_get_reg_u32(target, "pc", &pc);
	if (retval != ERROR_OK)
		return retval;
	if (pc != done_loop && pc != done_loop + 2) {
		LOG_ERROR("SNR8503x helper halted at 0x%08" PRIx32 ", expected BKPT at 0x%08" TARGET_PRIxADDR,
				pc, done_loop);
		return ERROR_TARGET_ALGO_EXIT;
	}

	retval = snr8503x_get_reg_u32(target, "r0", result);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

static int snr8503x_check_halted(struct target *target)
{
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

static int snr8503x_load_algo(struct target *target, struct working_area **algo_wa)
{
	int retval = target_alloc_working_area(target,
			sizeof(snr8503x_flash_algo) + SNR8503X_ALGO_TRAMPOLINE_SIZE, algo_wa);
	if (retval != ERROR_OK) {
		LOG_WARNING("no working area available for SNR8503x flash algorithm");
		return retval;
	}

	retval = snr8503x_write_buffer_retry(target, (*algo_wa)->address,
			sizeof(snr8503x_flash_algo), snr8503x_flash_algo);
	if (retval != ERROR_OK) {
		target_free_working_area(target, *algo_wa);
		*algo_wa = NULL;
		return retval;
	}

	target_addr_t trampoline_addr = snr8503x_algo_trampoline_addr(*algo_wa);
	uint8_t trampoline[SNR8503X_ALGO_TRAMPOLINE_SIZE] = {
		0x00, 0xbe, /* bkpt #0 */
		0xc0, 0x46, /* nop */
	};

	retval = snr8503x_write_buffer_retry(target,
			trampoline_addr, sizeof(trampoline), trampoline);
	if (retval != ERROR_OK) {
		target_free_working_area(target, *algo_wa);
		*algo_wa = NULL;
		return retval;
	}
	return ERROR_OK;
}

static int snr8503x_sector_erase(struct flash_bank *bank, unsigned int sector)
{
	struct target *target = bank->target;
	uint32_t value;
	int retval;
	bool save_poll_mask = jtag_poll_mask();

	retval = snr8503x_prepare(target);
	if (retval != ERROR_OK)
		goto out;

	retval = target_write_u32(target, SNR8503X_SYS_FLSE, SNR8503X_FLASH_ERASE_ENABLE);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_read_u32(target, SNR8503X_FLASH_CFG, &value);
	if (retval != ERROR_OK)
		goto cleanup;

	value &= ~(SNR8503X_CFG_CHIP_ERASE | SNR8503X_CFG_PROGRAM | SNR8503X_CFG_NVR);
	retval = target_write_u32(target, SNR8503X_FLASH_CFG, value);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, SNR8503X_FLASH_ADDR,
			bank->base + bank->sectors[sector].offset);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, SNR8503X_FLASH_CFG, value | SNR8503X_CFG_SECTOR_ERASE);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, SNR8503X_FLASH_ERASE, SNR8503X_FLASH_ERASE_KEY);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = snr8503x_wait_ready(target, SNR8503X_TIMEOUT_MS);

cleanup:
	snr8503x_cleanup(target);
out:
	jtag_poll_unmask(save_poll_mask);
	return retval;
}

static int snr8503x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	int retval = snr8503x_check_halted(bank->target);
	if (retval != ERROR_OK)
		return retval;

	if (first == 0 && last + 1 == bank->num_sectors)
		return snr8503x_mass_erase(bank);

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = snr8503x_sector_erase(bank, sector);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[sector].is_erased = 1;
	}

	return ERROR_OK;
}

static int snr8503x_write_host(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t cfg;
	uint32_t address = bank->base + offset;
	int retval;
	bool save_poll_mask = jtag_poll_mask();

	retval = snr8503x_prepare(target);
	if (retval != ERROR_OK)
		goto out;

	retval = target_write_u32(target, SNR8503X_SYS_FLSP, SNR8503X_FLASH_PROGRAM_ENABLE);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_read_u32(target, SNR8503X_FLASH_CFG, &cfg);
	if (retval != ERROR_OK)
		goto cleanup;

	cfg &= ~(SNR8503X_CFG_CHIP_ERASE | SNR8503X_CFG_SECTOR_ERASE | SNR8503X_CFG_NVR);
	cfg |= SNR8503X_CFG_PROGRAM;

	retval = target_write_u32(target, SNR8503X_FLASH_CFG, cfg);
	if (retval != ERROR_OK)
		goto cleanup;

	while (count > 0) {
		uint8_t word_bytes[4] = { 0xff, 0xff, 0xff, 0xff };
		uint32_t word;
		uint32_t chunk = MIN(count, 4U);

		memcpy(word_bytes, buffer, chunk);
		word = le_to_h_u32(word_bytes);

		retval = target_write_u32(target, SNR8503X_FLASH_ADDR, address);
		if (retval != ERROR_OK)
			goto cleanup;

		retval = target_write_u32(target, SNR8503X_FLASH_WDATA, word);
		if (retval != ERROR_OK)
			goto cleanup;

		retval = snr8503x_wait_ready(target, SNR8503X_TIMEOUT_MS);
		if (retval != ERROR_OK)
			goto cleanup;

		buffer += chunk;
		address += 4;
		count -= chunk;
		keep_alive();
	}

	retval = ERROR_OK;

cleanup:
	snr8503x_cleanup(target);
out:
	jtag_poll_unmask(save_poll_mask);
	return retval;
}

static int snr8503x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct working_area *algo_wa = NULL;
	struct working_area *data_wa = NULL;
	struct working_area *stack_wa = NULL;
	int retval;
	bool save_poll_mask = jtag_poll_mask();

	retval = snr8503x_check_halted(target);
	if (retval != ERROR_OK)
		goto out;

	if (offset + count > bank->size) {
		retval = ERROR_FLASH_DST_OUT_OF_BANK;
		goto out;
	}

	retval = snr8503x_load_algo(target, &algo_wa);
	if (retval != ERROR_OK)
		goto fallback_host;

	retval = target_alloc_working_area(target, SNR8503X_WRITE_CHUNK_SIZE, &data_wa);
	if (retval != ERROR_OK) {
		LOG_WARNING("no working area available for SNR8503x data buffer");
		goto fallback_host;
	}

	retval = target_alloc_working_area(target, SNR8503X_ALGO_STACK_SIZE, &stack_wa);
	if (retval != ERROR_OK) {
		LOG_WARNING("no working area available for SNR8503x algorithm stack");
		goto fallback_host;
	}

	retval = snr8503x_prepare(target);
	if (retval != ERROR_OK)
		goto cleanup_wa;

	uint32_t remaining = count;
	uint32_t address = bank->base + offset;
	uint8_t page_buffer[SNR8503X_WRITE_CHUNK_SIZE];

	while (remaining > 0) {
		uint32_t page_offset = address % SNR8503X_FLASH_SECTOR_SIZE;
		uint32_t page_remaining = SNR8503X_FLASH_SECTOR_SIZE - page_offset;
		uint32_t thisrun_count = MIN(remaining,
				MIN(page_remaining, (uint32_t)SNR8503X_WRITE_CHUNK_SIZE));
		uint32_t thisrun_padded = (thisrun_count + 3U) & ~3U;
		uint32_t result;

		memset(page_buffer, 0xff, sizeof(page_buffer));
		memcpy(page_buffer, buffer, thisrun_count);

		retval = snr8503x_write_buffer_retry(target, data_wa->address, thisrun_padded,
				page_buffer);
		if (retval != ERROR_OK)
			break;

		retval = snr8503x_run_algo(target,
				(algo_wa->address + SNR8503X_ALGO_PROGRAM_PAGE_OFFSET) | 1U,
				snr8503x_algo_trampoline_addr(algo_wa),
				snr8503x_algo_done_loop_addr(algo_wa),
				stack_wa->address + stack_wa->size,
				data_wa->address, address, thisrun_padded, &result);
		if (retval != ERROR_OK)
			break;

		if (result != 0) {
			LOG_ERROR("SNR8503x ProgramPage algorithm failed at 0x%08" PRIx32,
					address);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count;
		address += thisrun_count;
		remaining -= thisrun_count;
		alive_sleep(1);
	}

	if (retval == ERROR_OK)
		retval = snr8503x_cleanup(target);
	else
		snr8503x_cleanup(target);

cleanup_wa:
	if (stack_wa)
		target_free_working_area(target, stack_wa);
	if (data_wa)
		target_free_working_area(target, data_wa);
	if (algo_wa)
		target_free_working_area(target, algo_wa);
out:
	jtag_poll_unmask(save_poll_mask);
	return retval;

fallback_host:
	if (stack_wa) {
		target_free_working_area(target, stack_wa);
		stack_wa = NULL;
	}
	if (data_wa) {
		target_free_working_area(target, data_wa);
		data_wa = NULL;
	}
	if (algo_wa) {
		target_free_working_area(target, algo_wa);
		algo_wa = NULL;
	}
	retval = snr8503x_write_host(bank, buffer, offset, count);
	goto out;
}

static int snr8503x_probe(struct flash_bank *bank)
{
	free(bank->sectors);

	if (!bank->size)
		bank->size = SNR8503X_FLASH_SIZE;

	bank->write_start_alignment = 4;
	bank->write_end_alignment = 4;
	bank->default_padded_value = bank->erased_value = 0xff;

	bank->num_sectors = bank->size / SNR8503X_FLASH_SECTOR_SIZE;
	bank->sectors = alloc_block_array(0, SNR8503X_FLASH_SECTOR_SIZE, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = 0;

	return ERROR_OK;
}

static int snr8503x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t value;
	uint32_t fail;
	int retval;
	bool save_poll_mask = jtag_poll_mask();

	retval = snr8503x_check_halted(target);
	if (retval != ERROR_OK)
		goto out;

	retval = snr8503x_prepare(target);
	if (retval != ERROR_OK)
		goto out;

	retval = target_write_u32(target, SNR8503X_SYS_FLSE, SNR8503X_FLASH_ERASE_ENABLE);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_read_u32(target, SNR8503X_FLASH_CFG, &value);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, SNR8503X_FLASH_CFG, value | SNR8503X_CFG_CHIP_ERASE);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, SNR8503X_FLASH_ERASE, SNR8503X_FLASH_ERASE_KEY);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = snr8503x_wait_ready(target, SNR8503X_MASS_ERASE_TIMEOUT_MS);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = snr8503x_read_u32_retry(target, SNR8503X_FLASH_PROTECT, &fail,
			SNR8503X_TIMEOUT_MS);
	if (retval != ERROR_OK)
		goto cleanup;

	/*
	 * The FLM disassembly appears to treat 1 as success, but on real hardware
	 * the recovery sequence consistently completes with READY=1 and STATUS=0.
	 * The external recovery note for this chip also expects STATUS/FALL=0.
	 * Accept both values here and only reject unexpected ones.
	 */
	if (fail != 0 && fail != 1) {
		LOG_ERROR("SNR8503x mass erase reported unexpected status=0x%08" PRIx32,
				fail);
		retval = ERROR_FAIL;
	} else {
		LOG_DEBUG("SNR8503x mass erase status=0x%08" PRIx32, fail);
	}

cleanup:
	if (retval == ERROR_OK)
		retval = snr8503x_cleanup(target);
	else
		snr8503x_cleanup(target);
out:
	jtag_poll_unmask(save_poll_mask);
	return retval;
}

COMMAND_HANDLER(snr8503x_handle_mass_erase_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = snr8503x_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "SNR8503x mass erase complete");
	else
		command_print(CMD, "SNR8503x mass erase failed");

	return retval;
}

FLASH_BANK_COMMAND_HANDLER(snr8503x_flash_bank_command)
{
	return snr8503x_probe(bank);
}

static const struct command_registration snr8503x_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = snr8503x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire SNR8503x flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration snr8503x_command_handlers[] = {
	{
		.name = "snr8503x",
		.mode = COMMAND_ANY,
		.help = "SNR8503x flash command group",
		.usage = "",
		.chain = snr8503x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver snr8503x_flash = {
	.name = "snr8503x",
	.commands = snr8503x_command_handlers,
	.flash_bank_command = snr8503x_flash_bank_command,
	.erase = snr8503x_erase,
	.write = snr8503x_write,
	.read = default_flash_read,
	.probe = snr8503x_probe,
	.auto_probe = snr8503x_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
