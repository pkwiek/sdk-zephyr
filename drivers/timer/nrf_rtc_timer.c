/*
 * Copyright (c) 2016-2017 Nordic Semiconductor ASA
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <drivers/timer/system_timer.h>
#include <drivers/timer/nrf_rtc_timer.h>
#include <sys_clock.h>
#include <hal/nrf_rtc.h>
#include <spinlock.h>


#define EXT_CHAN_COUNT CONFIG_NRF_RTC_TIMER_USER_CHAN_COUNT
#define CHAN_COUNT (EXT_CHAN_COUNT + 1)

#define RTC NRF_RTC1
#define RTC_IRQn NRFX_IRQ_NUMBER_GET(RTC)
#define RTC_LABEL rtc1
#define RTC_CH_COUNT RTC1_CC_NUM

BUILD_ASSERT(CHAN_COUNT <= RTC_CH_COUNT, "Not enough compare channels");

#define COUNTER_BIT_WIDTH 24U
#define COUNTER_SPAN BIT(COUNTER_BIT_WIDTH)
#define COUNTER_MAX (COUNTER_SPAN - 1U)
#define COUNTER_HALF_SPAN (COUNTER_SPAN / 2U)
#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec()	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS ((COUNTER_HALF_SPAN - CYC_PER_TICK) / CYC_PER_TICK)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)

static struct k_spinlock lock;

static uint32_t last_count;

struct z_nrf_rtc_timer_chan_data {
	z_nrf_rtc_timer_compare_handler_t callback;
	void *user_context;
};

static struct z_nrf_rtc_timer_chan_data cc_data[CHAN_COUNT];
static uint32_t int_mask;
static atomic_t alloc_mask;

static volatile uint32_t m_overflow_cnt;
static volatile uint8_t  m_mutex;

static uint32_t counter_sub(uint32_t a, uint32_t b)
{
	return (a - b) & COUNTER_MAX;
}

static void set_comparator(int32_t chan, uint32_t cyc)
{
	nrf_rtc_cc_set(RTC, chan, cyc & COUNTER_MAX);
}

static uint32_t get_comparator(int32_t chan)
{
	return nrf_rtc_cc_get(RTC, chan);
}

static void event_clear(int32_t chan)
{
	nrf_rtc_event_clear(RTC, RTC_CHANNEL_EVENT_ADDR(chan));
}

static void event_enable(int32_t chan)
{
	nrf_rtc_event_enable(RTC, RTC_CHANNEL_INT_MASK(chan));
}

static void event_disable(int32_t chan)
{
	nrf_rtc_event_disable(RTC, RTC_CHANNEL_INT_MASK(chan));
}

static uint32_t counter(void)
{
	return nrf_rtc_counter_get(RTC);
}

static inline bool mutex_get(void)
{
	do
	{
		volatile uint8_t mutex_value = __LDREXB(&m_mutex);

		if (mutex_value)
		{
			__CLREX();
			return false;
		}
	}
	while (__STREXB(1, &m_mutex));

	/* Disable OVERFLOW interrupt to prevent lock-up in interrupt context
	 * while mutex is locked from lower priority context and OVERFLOW event
	 * flag is still up.
	 *
	 * This line should not be placed before acquiring the mutex to avoid a
	 * situation where interrupts are disabled unconditionally despite the
	 * mutex being locked. If executed by a high priority interrupt that
	 * could lead to preempting lower priority context releasing the mutex
	 * which would result in OVERFLOW interrupts being permanently disabled.
	 */
	nrf_rtc_int_disable(RTC, NRF_RTC_INT_OVERFLOW_MASK);

	__DMB();

	return true;
}

/** @brief Release mutex. */
static inline void mutex_release(void)
{
	/* Re-enable OVERFLOW interrupt. */
	nrf_rtc_int_enable(RTC, NRF_RTC_INT_OVERFLOW_MASK);

	__DMB();
	m_mutex = 0;
}

static uint32_t overflow_counter_get(void)
{
	uint32_t overflow;

	/* Get mutual access for writing to m_overflow_cnt variable. */
	if (mutex_get())
	{
		bool increasing = false;

		/* Check if interrupt was handled already. */
		if (nrf_rtc_event_check(RTC, NRF_RTC_EVENT_OVERFLOW))
		{
			m_overflow_cnt++;
			increasing = true;

			__DMB();

			/* Mark that interrupt was handled. */
			nrf_rtc_event_clear(RTC, NRF_RTC_EVENT_OVERFLOW);

			/* Result should be incremented. m_overflow_cnt will
			 * be incremented after mutex is released. */
		}
		else
		{
			/* Either overflow handling is not needed OR we acquired
			 * the mutex just after it was released. Overflow is
			 * handled after mutex is released, but it cannot be
			 * assured that m_overflow_cnt was incremented for the
			 * second time, so we increment the result here. */
		}

		overflow = (m_overflow_cnt + 1) / 2;

		mutex_release();

		if (increasing)
		{
			/* It's virtually impossible that overflow event is
			 * pending again before next instruction is performed.
			 * It is an error condition. */
			__ASSERT_NO_MSG(m_overflow_cnt & 0x01);

			/* Increment the counter for the second time, to allow
			 * instructions from other context get correct value of
			 * the counter. */
			m_overflow_cnt++;
		}
	}
	else
	{
		/* Failed to acquire mutex. */
		if (nrf_rtc_event_check(RTC, NRF_RTC_EVENT_OVERFLOW) ||
			(m_overflow_cnt & 0x01))
		{
			/* Lower priority context is currently incrementing
			 * m_overflow_cnt variable. */
			overflow = (m_overflow_cnt + 2) / 2;
		}
		else
		{
			/* Lower priority context has already incremented
			 * m_overflow_cnt variable or incrementing is not needed now. */
			overflow = m_overflow_cnt / 2;
		}
	}

	return overflow;
}

static void overflow_and_counter_get(uint32_t * p_overflow, uint32_t * p_counter)
{
	uint32_t overflow_1 = overflow_counter_get();

	__DMB();

	uint32_t rtc_value_1 = counter();

	__DMB();

	uint32_t overflow_2 = overflow_counter_get();

	*p_overflow  = overflow_2;
	*p_counter = (overflow_1 == overflow_2) ? rtc_value_1 : counter();
}

static uint32_t target_time_to_cc(uint64_t target_time)
{
	/* 24 least significant bits represent target CC value */
	return target_time & COUNTER_MAX;
}

static uint64_t overflow_and_counter_to_target_time(uint32_t overflow,
	uint32_t counter)
{
	return (((uint64_t)overflow) << COUNTER_BIT_WIDTH) | counter;
}

uint64_t z_nrf_rtc_timer_read(void)
{
	uint32_t overflow;
	uint32_t counter;

	overflow_and_counter_get(&overflow, &counter);

	return overflow_and_counter_to_target_time(overflow, counter);
}

uint32_t z_nrf_rtc_timer_compare_evt_address_get(int32_t chan)
{
	__ASSERT_NO_MSG(chan < CHAN_COUNT);
	return nrf_rtc_event_address_get(RTC, nrf_rtc_compare_event_get(chan));
}

bool z_nrf_rtc_timer_compare_int_lock(int32_t chan)
{
	__ASSERT_NO_MSG(chan && chan < CHAN_COUNT);

	uint32_t mcu_critical_state = __get_PRIMASK();
	__disable_irq();

	uint32_t prev = int_mask;
	int_mask &= ~BIT(chan);

	__set_PRIMASK(mcu_critical_state);

	nrf_rtc_int_disable(RTC, RTC_CHANNEL_INT_MASK(chan));

	return prev & BIT(chan);
}

void z_nrf_rtc_timer_compare_int_unlock(int32_t chan, bool key)
{
	__ASSERT_NO_MSG(chan && chan < CHAN_COUNT);

	if (key) {
		uint32_t mcu_critical_state = __get_PRIMASK();
		__disable_irq();

		int_mask |= BIT(chan);

		__set_PRIMASK(mcu_critical_state);

		nrf_rtc_int_enable(RTC, RTC_CHANNEL_INT_MASK(chan));
	}
}

uint32_t z_nrf_rtc_timer_compare_read(int32_t chan)
{
	__ASSERT_NO_MSG(chan < CHAN_COUNT);

	return nrf_rtc_cc_get(RTC, chan);
}

int z_nrf_rtc_timer_get_ticks(k_timeout_t t)
{
	uint32_t curr_count;
	int64_t curr_tick;
	int64_t result;
	int64_t abs_ticks;

	do {
		curr_count = counter();
		curr_tick = sys_clock_tick_get();
	} while (curr_count != counter());

	abs_ticks = Z_TICK_ABS(t.ticks);
	if (abs_ticks < 0) {
		/* relative timeout */
		return (t.ticks > COUNTER_HALF_SPAN) ?
			-EINVAL : ((curr_count + t.ticks) & COUNTER_MAX);
	}

	/* absolute timeout */
	result = abs_ticks - curr_tick;

	if ((result > COUNTER_HALF_SPAN) ||
	    (result < -(int64_t)COUNTER_HALF_SPAN)) {
		return -EINVAL;
	}

	return (curr_count + result) & COUNTER_MAX;
}

/* Function safely sets absolute alarm. It assumes that provided value is
 * less than COUNTER_HALF_SPAN from now. It detects late setting and also
 * handle +1 cycle case.
 */
static void set_absolute_alarm(int32_t chan, uint32_t abs_val)
{
	uint32_t now;
	uint32_t now2;
	uint32_t cc_val = abs_val & COUNTER_MAX;
	uint32_t prev_cc = get_comparator(chan);

	do {
		now = counter();

		/* Handle case when previous event may generate an event.
		 * It is handled by setting CC to now (far in the future),
		 * in case previous event was set for next tick wait for half
		 * LF tick and clear event that may have been generated.
		 */
		set_comparator(chan, now);
		if (counter_sub(prev_cc, now) == 1) {
			/* It should wait for half of RTC tick 15.26us. As
			 * busy wait runs from different clock source thus
			 * wait longer to cover for discrepancy.
			 */
			k_busy_wait(19);
		}


		/* If requested cc_val is in the past or next tick, set to 2
		 * ticks from now. RTC may not generate event if CC is set for
		 * 1 tick from now.
		 */
		if (counter_sub(cc_val, now + 2) > COUNTER_HALF_SPAN) {
			cc_val = now + 2;
		}

		event_clear(chan);
		event_enable(chan);
		set_comparator(chan, cc_val);
		now2 = counter();
		prev_cc = cc_val;
		/* Rerun the algorithm if counter progressed during execution
		 * and cc_val is in the past or one tick from now. In such
		 * scenario, it is possible that event will not be generated.
		 * Reruning the algorithm will delay the alarm but ensure that
		 * event will be generated at the moment indicated by value in
		 * CC register.
		 */
	} while ((now2 != now) &&
		 (counter_sub(cc_val, now2 + 2) > COUNTER_HALF_SPAN));
}

static void compare_set(int32_t chan, uint32_t cc_value,
			z_nrf_rtc_timer_compare_handler_t handler,
			void *user_data)
{
	cc_data[chan].callback = handler;
	cc_data[chan].user_context = user_data;

	set_absolute_alarm(chan, cc_value);
}

void z_nrf_rtc_timer_set(int32_t chan, uint64_t target_time,
			 z_nrf_rtc_timer_compare_handler_t handler,
			 void *user_data)
{
	__ASSERT_NO_MSG(chan && chan < CHAN_COUNT);

	bool key = z_nrf_rtc_timer_compare_int_lock(chan);

	uint32_t cc_value = target_time_to_cc(target_time);

	compare_set(chan, cc_value, handler, user_data);

	z_nrf_rtc_timer_compare_int_unlock(chan, key);
}

static void sys_clock_timeout_handler(int32_t chan,
				      uint64_t cc_value,
				      void *user_data)
{
	uint32_t dticks = counter_sub(cc_value, last_count) / CYC_PER_TICK;

	last_count += dticks * CYC_PER_TICK;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		/* protection is not needed because we are in the RTC interrupt
		 * so it won't get preempted by the interrupt.
		 */
		compare_set(chan, last_count + CYC_PER_TICK,
			    sys_clock_timeout_handler, NULL);
	}

	sys_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ?
						dticks : (dticks > 0));
}

/* Note: this function has public linkage, and MUST have this
 * particular name.  The platform architecture itself doesn't care,
 * but there is a test (tests/arch/arm_irq_vector_table) that needs
 * to find it to it can set it in a custom vector table.  Should
 * probably better abstract that at some point (e.g. query and reset
 * it by pointer at runtime, maybe?) so we don't have this leaky
 * symbol.
 */
void rtc_nrf_isr(const void *arg)
{
	ARG_UNUSED(arg);

	if (nrf_rtc_int_enable_check(RTC, NRF_RTC_INT_OVERFLOW_MASK) &&
		nrf_rtc_event_check(RTC, NRF_RTC_EVENT_OVERFLOW))
	{
		/* If OVERFLOW event becomes active during processing of
		 * overflow_counter_get() in a lower priority context, it is
		 * possible to preempt the lower priority context after it takes
		 * the mutex and before it disables the OVERFLOW interrupt. This
		 * situation would lead to a lock-up in interrupt context. It is
		 * caused by OVERFLOW interrupt enable/disable mechanism - refer
		 * to mutex_get() and mutex_release() for more information.
		 * In the described case the interrupt context would be unable
		 * to acquire the mutex, this means it would be unable to clear
		 * the OVERFLOW flag or disable the OVERFLOW interrupt as well.
		 * Leaving it continuously pending.
		 * OVERFLOW interrupt will be re-enabled when mutex is released
		 * - either from this handler, or from lower priority context
		 * that locked the mutex. */
		nrf_rtc_int_disable(RTC, NRF_RTC_INT_OVERFLOW_MASK);

		(void)overflow_counter_get();
	}

	for (int32_t chan = 0; chan < CHAN_COUNT; chan++) {
		if (nrf_rtc_int_enable_check(RTC, RTC_CHANNEL_INT_MASK(chan)) &&
		    nrf_rtc_event_check(RTC, RTC_CHANNEL_EVENT_ADDR(chan))) {
			uint32_t cc_val;
			uint32_t now;
			uint32_t overflow;
			uint64_t time;
			z_nrf_rtc_timer_compare_handler_t handler;

			event_clear(chan);
			event_disable(chan);
			cc_val = get_comparator(chan);
			overflow_and_counter_get(&overflow, &now);

			/* Higher priority interrupt may already changed cc_val
			 * which now points to the future. In that case return
			 * current counter value. It is less precise than
			 * returning exact CC value but this one is already lost.
			 */
			if (counter_sub(now, cc_val) > COUNTER_HALF_SPAN) {
				cc_val = now;
			}

			time = overflow_and_counter_to_target_time(overflow,
								   cc_val);

			handler = cc_data[chan].callback;
			cc_data[chan].callback = NULL;
			if (handler) {
				handler(chan, time, cc_data[chan].user_context);
			}
		}
	}
}

int32_t z_nrf_rtc_timer_chan_alloc(void)
{
	int32_t chan;
	atomic_val_t prev;
	do {
		chan = alloc_mask ? 31 - __builtin_clz(alloc_mask) : -1;
		if (chan < 0) {
			return -ENOMEM;
		}
		prev = atomic_and(&alloc_mask, ~BIT(chan));
	} while (!(prev & BIT(chan)));

	return chan;
}

void z_nrf_rtc_timer_chan_free(int32_t chan)
{
	__ASSERT_NO_MSG(chan && chan < CHAN_COUNT);

	atomic_or(&alloc_mask, BIT(chan));
}

int sys_clock_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	static const enum nrf_lfclk_start_mode mode =
		IS_ENABLED(CONFIG_SYSTEM_CLOCK_NO_WAIT) ?
			CLOCK_CONTROL_NRF_LF_START_NOWAIT :
			(IS_ENABLED(CONFIG_SYSTEM_CLOCK_WAIT_FOR_AVAILABILITY) ?
			CLOCK_CONTROL_NRF_LF_START_AVAILABLE :
			CLOCK_CONTROL_NRF_LF_START_STABLE);

	/* TODO: replace with counter driver to access RTC */
	nrf_rtc_prescaler_set(RTC, 0);
	for (int32_t chan = 0; chan < CHAN_COUNT; chan++) {
		nrf_rtc_int_enable(RTC, RTC_CHANNEL_INT_MASK(chan));
	}

	NVIC_ClearPendingIRQ(RTC_IRQn);

	IRQ_CONNECT(RTC_IRQn, DT_IRQ(DT_NODELABEL(RTC_LABEL), priority),
		    rtc_nrf_isr, 0, 0);
	irq_enable(RTC_IRQn);

	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_CLEAR);
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_START);

	int_mask = BIT_MASK(CHAN_COUNT);
	if (CONFIG_NRF_RTC_TIMER_USER_CHAN_COUNT) {
		alloc_mask = BIT_MASK(EXT_CHAN_COUNT) << 1;
	}

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		compare_set(0, counter() + CYC_PER_TICK,
			    sys_clock_timeout_handler, NULL);
	}

	z_nrf_clock_control_lf_on(mode);

	return 0;
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);
	uint32_t cyc;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	uint32_t unannounced = counter_sub(counter(), last_count);

	/* If we haven't announced for more than half the 24-bit wrap
	 * duration, then force an announce to avoid loss of a wrap
	 * event.  This can happen if new timeouts keep being set
	 * before the existing one triggers the interrupt.
	 */
	if (unannounced >= COUNTER_HALF_SPAN) {
		ticks = 0;
	}

	/* Get the cycles from last_count to the tick boundary after
	 * the requested ticks have passed starting now.
	 */
	cyc = ticks * CYC_PER_TICK + 1 + unannounced;
	cyc += (CYC_PER_TICK - 1);
	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;

	/* Due to elapsed time the calculation above might produce a
	 * duration that laps the counter.  Don't let it.
	 */
	if (cyc > MAX_CYCLES) {
		cyc = MAX_CYCLES;
	}

	cyc += last_count;
	compare_set(0, cyc, sys_clock_timeout_handler, NULL);
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	return counter_sub(counter(), last_count) / CYC_PER_TICK;
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = counter_sub(counter(), last_count) + last_count;

	k_spin_unlock(&lock, key);
	return ret;
}
