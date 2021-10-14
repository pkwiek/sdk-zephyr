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

#define ANCHOR_RANGE_START (COUNTER_SPAN / 8)
#define ANCHOR_RANGE_END (7 * COUNTER_SPAN / 8)

static uint32_t overflow_cnt;
static atomic_t anchor_mutex;
static volatile uint64_t anchor;
static uint32_t last_count;

struct z_nrf_rtc_timer_chan_data {
	z_nrf_rtc_timer_compare_handler_t callback;
	void *user_context;
	uint32_t target_cc;
	uint64_t target_time;
};

static struct z_nrf_rtc_timer_chan_data cc_data[CHAN_COUNT];
static atomic_t int_mask;
static atomic_t alloc_mask;
static atomic_t force_isr_mask;

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

static uint32_t absolute_time_to_cc(uint64_t absolute_time)
{
	/* 24 least significant bits represent target CC value */
	return absolute_time & COUNTER_MAX;
}

uint32_t z_nrf_rtc_timer_compare_evt_address_get(int32_t chan)
{
	__ASSERT_NO_MSG(chan < CHAN_COUNT);
	return nrf_rtc_event_address_get(RTC, nrf_rtc_compare_event_get(chan));
}

bool z_nrf_rtc_timer_compare_int_lock(int32_t chan)
{
	__ASSERT_NO_MSG(chan && chan < CHAN_COUNT);

	atomic_val_t prev = atomic_and(&int_mask, ~BIT(chan));

	nrf_rtc_int_disable(RTC, RTC_CHANNEL_INT_MASK(chan));

	return prev & BIT(chan);
}

void z_nrf_rtc_timer_compare_int_unlock(int32_t chan, bool key)
{
	__ASSERT_NO_MSG(chan && chan < CHAN_COUNT);

	if (key) {
		atomic_or(&int_mask, BIT(chan));
		nrf_rtc_int_enable(RTC, RTC_CHANNEL_INT_MASK(chan));
		if (atomic_get(&force_isr_mask) & BIT(chan)) {
			NVIC_SetPendingIRQ(RTC_IRQn);
		}
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
static uint32_t set_absolute_alarm(int32_t chan, uint32_t abs_val)
{
	uint32_t now;
	uint32_t now2;
	uint32_t cc_val = abs_val & COUNTER_MAX;
	uint32_t prev_cc = get_comparator(chan);

	do {
		now = counter();

		/* A case when previous CC value may generate an event is not
		 * handled here. It is handled in RTC's ISR handler by filtering
		 * past events based on target CC value.
		 */

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
		 * Rerunning the algorithm will delay the alarm but ensure that
		 * event will be generated at the moment indicated by value in
		 * CC register.
		 */
	} while ((now2 != now) &&
		 (counter_sub(cc_val, now2 + 2) > COUNTER_HALF_SPAN));

	return cc_val;
}

/* This function operates on 32-bit values. It can be used directly when
 * 64-bit absolute time is not required.
 */
static void compare_set(int32_t chan, uint32_t cc_value,
			z_nrf_rtc_timer_compare_handler_t handler,
			void *user_data)
{
	cc_data[chan].callback = handler;
	cc_data[chan].user_context = user_data;
	cc_data[chan].target_cc = set_absolute_alarm(chan, cc_value);
}

int z_nrf_rtc_timer_set(int32_t chan, uint64_t target_time,
			 z_nrf_rtc_timer_compare_handler_t handler,
			 void *user_data)
{
	__ASSERT_NO_MSG(chan && chan < CHAN_COUNT);

	bool key;
	int ret = 0;
	uint32_t cc_value = absolute_time_to_cc(target_time);
	uint64_t curr_time = z_nrf_rtc_timer_read();

	key = z_nrf_rtc_timer_compare_int_lock(chan);

	if (curr_time < target_time) {
		if (target_time - curr_time > COUNTER_SPAN) {
			/* Target time is too distant. */
			ret = -EINVAL;
		} else if (target_time != cc_data[chan].target_time) {
			/* Target time is valid. Set CC value. */
			compare_set(chan, cc_value, handler, user_data);

			/* Update the absolute expiration time based on
			 * CC value that was set.
			 */
			uint64_t set_time = target_time & ~COUNTER_MAX;

			set_time += cc_data[chan].target_cc;

			if (cc_value > cc_data[chan].target_cc) {
				/* CC value was set after an overflow. Correct
				 * target_time by a counter overflow value.
				 */
				set_time += COUNTER_SPAN;
			}

			cc_data[chan].target_time = set_time;
		} else {
			/* Provided target time was already set. */
			cc_data[chan].callback = handler;
			cc_data[chan].user_context = user_data;
		}
	} else {
		cc_data[chan].callback = handler;
		cc_data[chan].user_context = user_data;
		cc_data[chan].target_time = target_time;
		cc_data[chan].target_cc = cc_value;

		/* Force ISR hadling when exiting from critical section. */
		atomic_or(&force_isr_mask, BIT(chan));
	}

	z_nrf_rtc_timer_compare_int_unlock(chan, key);

	return ret;
}

void z_nrf_rtc_timer_abort(int32_t chan)
{
	__ASSERT_NO_MSG(chan && chan < CHAN_COUNT);

	bool key = z_nrf_rtc_timer_compare_int_lock(chan);

	event_clear(chan);
	event_disable(chan);

	z_nrf_rtc_timer_compare_int_unlock(chan, key);
}

uint64_t z_nrf_rtc_timer_read(void)
{
	uint64_t val = (overflow_cnt << COUNTER_BIT_WIDTH);

	__DMB();

	val += counter();

	/* The values of overflow counter and RTC counter have been read.
	 * Now verify that they are valid.
	 */
	if (atomic_get(&anchor_mutex) == 0) {
		/* 64-bit load and stores are not atomic. Additional variable
		 * must be used to detect anchor update happening in the meantime.
		 */
		uint64_t tmp_anchor = anchor;

		__DMB();

		/* If anchor has changed then this function was preempted by updating
		 * anchor. However, anchor is only updated when we are far from
		 * overflow, so current overflow counter is certainly valid.
		 *
		 * If the anchor has not changed then there's a possibility that
		 * we are close to overflow. Detect whether overflow happened
		 * between loading current overflow counter and RTC counter - in
		 * that case, the anchor contains the same value of overflow counter,
		 * but larger RTC counter (24 least significant bits), and therefore
		 * is larger.
		 */
		if (anchor == tmp_anchor && val < tmp_anchor) {
			val += COUNTER_SPAN;
		}
	} else {
		/* If the mutex could not be acquired, this function interrupted
		 * storing anchor. Therefore we are far from overflow, so there's
		 * no need to compare against anchor - current overflow counter
		 * is certainly valid.
		 */
	}

	return val;
}

static inline bool in_anchor_range(uint32_t cc_value)
{
	return (cc_value >= ANCHOR_RANGE_START) && (cc_value < ANCHOR_RANGE_END);
}

static inline bool anchor_update(uint32_t cc_value)
{
	/* Update anchor when far from overflow */
	if (in_anchor_range(cc_value)) {
		atomic_set(&anchor_mutex, 1);
		anchor = (overflow_cnt << COUNTER_BIT_WIDTH) + cc_value;
		atomic_set(&anchor_mutex, 0);
		return true;
	}

	return false;
}

static void sys_clock_timeout_handler(int32_t chan,
				      uint64_t expire_time,
				      void *user_data)
{
	uint32_t cc_value = absolute_time_to_cc(expire_time);
	uint32_t dticks = counter_sub(cc_value, last_count) / CYC_PER_TICK;

	last_count += dticks * CYC_PER_TICK;

	bool anchor_updated = anchor_update(cc_value);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		/* protection is not needed because we are in the RTC interrupt
		 * so it won't get preempted by the interrupt.
		 */
		compare_set(chan, last_count + CYC_PER_TICK,
					  sys_clock_timeout_handler, NULL);
	}

	sys_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ?
						dticks : (dticks > 0));

	if (cc_value == get_comparator(chan)) {
		/* New value was not set. Set something that can update anchor.
		 * If anchor was updated we can enable same CC value to trigger
		 * interrupt after full cycle. Else set event in anchor update
		 * range. Since anchor was not updated we know that it's very
		 * far from mid point so setting is done without any protection.
		 */
		if (!anchor_updated) {
			set_comparator(chan, COUNTER_HALF_SPAN);
		}
		event_enable(chan);
	}
}

static bool channel_processing_check_and_clear(int32_t chan)
{
	bool result = false;
	uint32_t mcu_critical_state = __get_PRIMASK();

	__disable_irq();

	if (nrf_rtc_int_enable_check(RTC, RTC_CHANNEL_INT_MASK(chan))) {
		/* The processing of channel can be caused by CC match
		 * or be forced.
		 */
		result = atomic_and(&force_isr_mask, ~BIT(chan)) ||
			 nrf_rtc_event_check(RTC, RTC_CHANNEL_EVENT_ADDR(chan));

		if (result) {
			nrf_rtc_event_clear(RTC, RTC_CHANNEL_EVENT_ADDR(chan));
			nrf_rtc_event_disable(RTC, RTC_CHANNEL_INT_MASK(chan));
		}
	}

	__set_PRIMASK(mcu_critical_state);

	return result;
}

static uint32_t full_int_lock(void)
{
	uint32_t mcu_critical_state;

	if (IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS)) {
		mcu_critical_state = __get_PRIMASK();
		__disable_irq();
	} else {
		mcu_critical_state = irq_lock();
	}

	return mcu_critical_state;
}

static void full_int_unlock(uint32_t mcu_critical_state)
{
	if (IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS)) {
		__set_PRIMASK(mcu_critical_state);
	} else {
		irq_unlock(mcu_critical_state);
	}
}

static void process_channel(int32_t chan)
{
	if (channel_processing_check_and_clear(chan)) {
		uint32_t counter;
		void *user_context;
		uint32_t mcu_critical_state;
		uint64_t curr_time;
		uint64_t expire_time;
		z_nrf_rtc_timer_compare_handler_t handler = NULL;

		curr_time = z_nrf_rtc_timer_read();
		counter = absolute_time_to_cc(curr_time);

		/* This critical section is used to provide atomic access to
		 * cc_data structure and prevent higher priority contexts
		 * (including ZLIs) from overwriting it.
		 */
		mcu_critical_state = full_int_lock();

		/* If target_cc is in the past or is equal to current counter
		 * value, execute the handler.
		 */
		if ((cc_data[chan].target_time != 0U) &&
		    (curr_time >= cc_data[chan].target_time)) {
			/* Absolute time is used by this channel. */
			expire_time = cc_data[chan].target_time;

			/* Latch an reset handler data. */
			handler = cc_data[chan].callback;
			user_context = cc_data[chan].user_context;
			cc_data[chan].callback = NULL;
		} else if (counter_sub(counter, cc_data[chan].target_cc)
			 < COUNTER_HALF_SPAN) {
			/* Absolute time is not used by this channel.
			 * expire time is equivalent to target_cc value.
			 */
			expire_time = cc_data[chan].target_cc;

			/* Latch an reset handler data. */
			handler = cc_data[chan].callback;
			user_context = cc_data[chan].user_context;
			cc_data[chan].callback = NULL;
		}

		full_int_unlock(mcu_critical_state);

		if (handler) {
			handler(chan, expire_time, user_context);
		}
	}
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
	    nrf_rtc_event_check(RTC, NRF_RTC_EVENT_OVERFLOW)) {
		nrf_rtc_event_clear(RTC, NRF_RTC_EVENT_OVERFLOW);
		overflow_cnt++;
	}

	for (int32_t chan = 0; chan < CHAN_COUNT; chan++) {
		process_channel(chan);
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

	nrf_rtc_int_enable(RTC, NRF_RTC_INT_OVERFLOW_MASK);

	NVIC_ClearPendingIRQ(RTC_IRQn);

	IRQ_CONNECT(RTC_IRQn, DT_IRQ(DT_NODELABEL(RTC_LABEL), priority),
		    rtc_nrf_isr, 0, 0);
	irq_enable(RTC_IRQn);

	nrf_rtc_int_enable(RTC, NRF_RTC_INT_OVERFLOW_MASK);
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_CLEAR);
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_START);

	int_mask = BIT_MASK(CHAN_COUNT);
	if (CONFIG_NRF_RTC_TIMER_USER_CHAN_COUNT) {
		alloc_mask = BIT_MASK(EXT_CHAN_COUNT) << 1;
	}

	uint32_t initial_timeout = IS_ENABLED(CONFIG_TICKLESS_KERNEL) ?
				(COUNTER_HALF_SPAN - 1) :
				(counter() + CYC_PER_TICK);

	compare_set(0, initial_timeout, sys_clock_timeout_handler, NULL);

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
	return (uint32_t)z_nrf_rtc_timer_read();
}
