/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "platform/nrf_802154_platform_sl_lp_timer.h"

#include <assert.h>
#include <kernel.h>
#include <stdbool.h>

#include "drivers/timer/nrf_rtc_timer.h"

#include "platform/nrf_802154_clock.h"
#include "nrf_802154_sl_utils.h"

static volatile bool m_clock_ready;
static bool m_is_running;
static int32_t m_rtc_channel;
static bool m_in_critical_section;
static uint64_t m_last_scheduled_time;

void rtc_irq_handler(int32_t id, uint64_t target_time, void *user_data)
{
	(void)user_data;

	assert(id == m_rtc_channel);

	/*! TODO: Is this precise enough?
	 *  @todo Is this precise enough?
	 */
	bool was_running = m_is_running;
	m_is_running = false;

	/* Force to store M_is_running flag. */
	__DMB();

	if (was_running) 
	{
		nrf_802154_sl_timer_handler(target_time);
	}
}

void nrf_802154_clock_lfclk_ready(void)
{
	m_clock_ready = true;
}

void nrf_802154_platform_sl_lp_timer_init(void)
{
	m_in_critical_section = false;
	m_is_running = false;

	/* Setup low frequency clock. */
	nrf_802154_clock_lfclk_start();

	while (!m_clock_ready) {
		/* Intentionally empty */
	}

	m_rtc_channel = z_nrf_rtc_timer_chan_alloc();
	if (m_rtc_channel < 0) {
		assert(false);
		return;
	}

	(void)z_nrf_rtc_timer_compare_int_lock(m_rtc_channel);
}

void nrf_802154_platform_sl_lp_timer_deinit(void)
{
	(void)z_nrf_rtc_timer_compare_int_lock(m_rtc_channel);

	z_nrf_rtc_timer_chan_free(m_rtc_channel);

	nrf_802154_clock_lfclk_stop();
}

uint64_t nrf_802154_platform_sl_lptimer_current_lpticks_get(void)
{
	return z_nrf_rtc_timer_read();
}

/**@brief Converts time in microseconds to @c lpticks
 */
uint64_t nrf_802154_platform_sl_lptimer_us_to_lpticks_convert(uint64_t us, bool round_up)
{
    /* TODO */
}

/**@brief Converts time in @c lpticks to microseconds.
 */
uint64_t nrf_802154_platform_sl_lptimer_lpticks_to_us_convert(uint64_t lpticks)
{
    /* TODO */
}

/* TODO: Should schedule_at and disable functions be preemptible by each other?
 * This would mean guarding m_is_running flag.
 */
void nrf_802154_platform_sl_lptimer_schedule_at(uint64_t fire_lpticks);
{
	/* This function is not required to be reentrant, hence no critical section. */
	z_nrf_rtc_timer_set(m_rtc_channel, fire_lpticks, rtc_irq_handler, NULL);

	/* TODO: What if we are preempted here, m_is_running could be left unset leading to
	 * missing an event. Maybe an mcu critical section is still needed.
	 */
	m_last_scheduled_time = fire_lpticks;
	m_is_running = true;
}

void nrf_802154_platform_sl_lptimer_disable(void);
{
	/* This function is not required to be reentrant, hence no critical section. */
	m_is_running = false;
	__DMB();

	(void)z_nrf_rtc_timer_compare_int_lock(m_rtc_channel);
}

void nrf_802154_platform_sl_lptimer_critical_section_enter(void)
{
	nrf_802154_sl_mcu_critical_state_t state;

	nrf_802154_sl_mcu_critical_enter(state);

	if (!m_in_critical_section) {
		(void)z_nrf_rtc_timer_compare_int_lock(m_rtc_channel);
		m_in_critical_section = true;
	}

	nrf_802154_sl_mcu_critical_exit(state);
}

/**@brief Exits out of critical section of the lptimer module.
 */
void nrf_802154_platform_sl_lptimer_critical_section_exit(void)
{
	nrf_802154_sl_mcu_critical_state_t state;

	nrf_802154_sl_mcu_critical_enter(state);

	m_in_critical_section = false;

	z_nrf_rtc_timer_compare_int_unlock(m_rtc_channel, m_is_running);

	nrf_802154_sl_mcu_critical_exit(state);
}
