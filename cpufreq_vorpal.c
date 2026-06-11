// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/cpuidle.h>
#include <linux/sched/cpufreq.h>
#include <linux/sched/topology.h>
#include <linux/sched/rt.h>
#include <linux/sched/types.h>
#include <uapi/linux/sched/types.h>
#include <linux/timekeeping.h>
#include <linux/ktime.h>
#include <linux/atomic.h>
#include <linux/tick.h>
#include <linux/jiffies.h>
#include <linux/irq_work.h>
#include <linux/input.h>

extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);
extern int sched_gaming_active;

#define VORPAL_VER  "7.0"
#define VORPAL_AUTH "Templar Dev"

#define LITTLE_CAP_THR	614
#define PRIME_CAP_THR	1000

#define LITTLE_UP_US	0
#define LITTLE_DOWN_US	2000
#define BIG_UP_US	0
#define BIG_DOWN_US	3000
#define PRIME_UP_US	0
#define PRIME_DOWN_US	3000

#define GAMING_BAND_LITTLE_MIN	1000000
#define GAMING_BAND_LITTLE_MAX	1500000
#define GAMING_BAND_BIG_MIN	1800000
#define GAMING_BAND_BIG_MAX	2200000
#define GAMING_BAND_PRIME_MIN	2200000
#define GAMING_BAND_PRIME_MAX	2700000

#define GAMING_SCALE_LITTLE	8
#define GAMING_SCALE_BIG	10
#define GAMING_SCALE_PRIME	12

#define GAMING_LOCK_NS		(500 * NSEC_PER_MSEC)
#define GAMING_HYSTERESIS_NS	(2000 * NSEC_PER_MSEC)

#define THERMAL_CAP_MIN		50

#define FRAME_BUDGET_US		8333
#define FRAME_MISS_NUM		11
#define FRAME_MISS_DEN		10
#define FRAME_BOOST_NS		(50 * NSEC_PER_MSEC)
#define FRAME_FLOOR_PRIME_PCT	90
#define FRAME_FLOOR_BIG_PCT	85

#define FRAME_HIST_SIZE		16
#define FRAME_BOOST_BASE	0
#define FRAME_BOOST_LIGHT	5
#define FRAME_BOOST_MED		10
#define FRAME_BOOST_HEAVY	20

#define RENDER_DETECT_NS	(16 * NSEC_PER_MSEC)
#define RENDER_BOOST_NS		(24 * NSEC_PER_MSEC)
#define RENDER_BOOST_PCT	30
#define RENDER_CONF_COUNT	2

#define INTERACT_WINDOW_NS	(40 * NSEC_PER_MSEC)
#define INTERACT_LITTLE_KHZ	1000000
#define INTERACT_BIG_KHZ	1300000

#define IDLE_STALE_NS		(8 * NSEC_PER_MSEC)
#define IDLE_ENTER_PCT		1

#define INTERACTIVE_DUR_NS	(500 * NSEC_PER_MSEC)
#define INTERACTIVE_FLOOR_PCT	8

#define EMA_ALPHA		140
#define EMA_CLAMP		15

#define SUSTAIN_BASE_NS		(40 * NSEC_PER_MSEC)
#define SUSTAIN_FRAME_NS	(80 * NSEC_PER_MSEC)
#define SUSTAIN_DROP_THR	15

#define SCHED_FLAGS_UGOV	0x10000000
#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

enum rfx_cluster_type {
	CLUSTER_LITTLE = 0,
	CLUSTER_BIG    = 1,
	CLUSTER_PRIME  = 2,
};

struct rfx_policy;

struct rfx_tunables {
	struct gov_attr_set attr_set;
	unsigned int rate_limit_us;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
	enum rfx_cluster_type cluster_type;
};

struct rfx_cpu {
	struct update_util_data update_util;
	struct rfx_policy *rfx_policy;
	unsigned int cpu;
	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;
	unsigned long util;
	unsigned long bwmin;
	unsigned int prev_util_pct;
	unsigned int prev_util_pct2;
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

struct rfx_frame_hist {
	u64 timestamp;
	unsigned int frame_time;
	bool missed;
};

struct rfx_policy {
	struct cpufreq_policy *policy;
	struct rfx_tunables *tunables;
	struct list_head tunables_hook;

	raw_spinlock_t update_lock;
	u64 last_upfreq_time;
	u64 last_downfreq_time;
	s64 freq_update_delay_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;

	unsigned int next_freq;
	unsigned int cached_raw_freq;

	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;
	bool limits_changed;
	bool need_freq_update;

	u64 gaming_lock_end_ns;
	u64 frame_boost_end_ns;
	u64 sustain_hold_end_ns;
	u64 interactive_end_ns;
	u64 render_boost_end_ns;

	unsigned int thermal_cap_smoothed;
	u64 thermal_step_ns;

	bool in_deep_idle;
	u64 last_real_update_ns;
	bool in_heavy_mode;
	unsigned int ema_util_pct;
	unsigned int spike_confirm;
	unsigned int render_detect_count;
	u64 render_detect_start_ns;
	unsigned int frame_boost_level;
	u64 frame_boost_applied_ns;
};

static DEFINE_PER_CPU(struct rfx_cpu, rfx_cpu_table);

static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

static atomic_t rfx_gaming_mode = ATOMIC_INIT(0);
static atomic_t rfx_thermal_cap_pct = ATOMIC_INIT(100);
static atomic_t rfx_frame_time_us = ATOMIC_INIT(0);
static atomic_t rfx_frame_budget_us = ATOMIC_INIT(FRAME_BUDGET_US);
static atomic64_t rfx_input_ts_ns = ATOMIC64_INIT(0);
static atomic_t rfx_render_hint = ATOMIC_INIT(0);

static struct rfx_frame_hist rfx_frame_hist[FRAME_HIST_SIZE];
static unsigned int rfx_frame_hist_idx;
static DEFINE_SPINLOCK(rfx_frame_hist_lock);

static inline bool rfx_gaming(void)
{
	return atomic_read(&rfx_gaming_mode) != 0;
}

static inline bool rfx_is_little(unsigned long cap)
{
	return cap <= LITTLE_CAP_THR;
}

static inline bool rfx_is_prime(unsigned long cap)
{
	return cap >= PRIME_CAP_THR;
}

static inline unsigned int rfx_pct_of_max(struct cpufreq_policy *policy, unsigned int pct)
{
	if (!policy || !policy->cpuinfo.max_freq)
		return 0;
	return (unsigned int)((u64)policy->cpuinfo.max_freq * pct / 100);
}

static inline unsigned int rfx_floor_of_max(struct cpufreq_policy *policy, unsigned int pct)
{
	unsigned int f = rfx_pct_of_max(policy, pct);
	return max(f, policy->cpuinfo.min_freq);
}

static inline unsigned int rfx_abs(struct cpufreq_policy *policy, unsigned int khz)
{
	return clamp(khz, policy->cpuinfo.min_freq, policy->cpuinfo.max_freq);
}

static inline struct rfx_tunables *to_rfx_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct rfx_tunables, attr_set);
}

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bwmin))
		rfx_c->rfx_policy->need_freq_update = true;
}

static unsigned int rfx_ema_filter(struct rfx_policy *rfx_pol, unsigned int raw_pct)
{
	unsigned int ema = rfx_pol->ema_util_pct;

	ema = (ema * (256 - EMA_ALPHA) + raw_pct * EMA_ALPHA) / 256;

	if (ema < raw_pct && raw_pct - ema > EMA_CLAMP)
		ema = raw_pct - (EMA_CLAMP >> 1);
	if (ema > raw_pct && ema - raw_pct > EMA_CLAMP)
		ema = raw_pct + (EMA_CLAMP >> 1);

	rfx_pol->ema_util_pct = ema;
	return ema;
}

static unsigned int rfx_thermal_cap(struct rfx_policy *rfx_pol, u64 time)
{
	unsigned int target = atomic_read(&rfx_thermal_cap_pct);

	if (target > 100)
		target = 100;
	if (target < THERMAL_CAP_MIN)
		target = THERMAL_CAP_MIN;

	return target;
}

static void rfx_frame_hist_update(u64 time, unsigned int ft, unsigned int budget)
{
	unsigned long flags;
	bool missed = false;

	if (!ft || !budget)
		return;

	if ((u64)ft * FRAME_MISS_DEN > (u64)budget * FRAME_MISS_NUM)
		missed = true;

	spin_lock_irqsave(&rfx_frame_hist_lock, flags);
	rfx_frame_hist_idx = (rfx_frame_hist_idx + 1) % FRAME_HIST_SIZE;
	rfx_frame_hist[rfx_frame_hist_idx].timestamp = time;
	rfx_frame_hist[rfx_frame_hist_idx].frame_time = ft;
	rfx_frame_hist[rfx_frame_hist_idx].missed = missed;
	spin_unlock_irqrestore(&rfx_frame_hist_lock, flags);
}

static unsigned int rfx_frame_boost_level(void)
{
	unsigned int consecutive_miss = 0;
	unsigned int consecutive_ok = 0;
	unsigned int max_miss = 0;
	unsigned int max_ok = 0;
	unsigned int i;
	unsigned long flags;

	spin_lock_irqsave(&rfx_frame_hist_lock, flags);
	for (i = 0; i < FRAME_HIST_SIZE; i++) {
		unsigned int idx = (rfx_frame_hist_idx + FRAME_HIST_SIZE - i) % FRAME_HIST_SIZE;
		if (rfx_frame_hist[idx].missed) {
			consecutive_miss++;
			consecutive_ok = 0;
		} else {
			consecutive_ok++;
			consecutive_miss = 0;
		}
		if (consecutive_miss > max_miss)
			max_miss = consecutive_miss;
		if (consecutive_ok > max_ok)
			max_ok = consecutive_ok;
	}
	spin_unlock_irqrestore(&rfx_frame_hist_lock, flags);

	if (max_miss >= 3)
		return FRAME_BOOST_HEAVY;
	if (max_miss >= 1)
		return FRAME_BOOST_MED;
	if (max_ok >= 10)
		return FRAME_BOOST_BASE;
	return FRAME_BOOST_LIGHT;
}

static void rfx_frame_observe(struct rfx_policy *rfx_pol, u64 time)
{
	unsigned int ft = atomic_read(&rfx_frame_time_us);
	unsigned int budget = atomic_read(&rfx_frame_budget_us);
	unsigned int level;

	if (!rfx_gaming() || !ft || !budget)
		return;

	rfx_frame_hist_update(time, ft, budget);

	if ((u64)ft * FRAME_MISS_DEN > (u64)budget * FRAME_MISS_NUM) {
		rfx_pol->frame_boost_end_ns = time + FRAME_BOOST_NS;
		level = rfx_frame_boost_level();
		if (level != rfx_pol->frame_boost_level) {
			rfx_pol->frame_boost_level = level;
			rfx_pol->frame_boost_applied_ns = time;
		}
	}
}

static bool rfx_iowait_reset(struct rfx_cpu *rfx_c, u64 time, bool set_boost)
{
	s64 delta = time - rfx_c->last_update;

	if (delta <= TICK_NSEC)
		return false;
	rfx_c->iowait_boost = set_boost ? IOWAIT_BOOST_MIN : 0;
	rfx_c->iowait_boost_pending = set_boost;
	return true;
}

static void rfx_iowait_boost(struct rfx_cpu *rfx_c, u64 time, unsigned int flags)
{
	bool set_boost = flags & SCHED_CPUFREQ_IOWAIT;
	unsigned long max_cap;
	unsigned int cap;

	if (rfx_gaming())
		return;

	if (rfx_c->iowait_boost) {
		if (!rfx_iowait_reset(rfx_c, time, set_boost))
			rfx_c->iowait_boost_pending = set_boost;
		return;
	}
	if (!set_boost || rfx_c->iowait_boost_pending)
		return;

	rfx_c->iowait_boost_pending = true;
	max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	cap = rfx_is_little(max_cap) ? (SCHED_CAPACITY_SCALE / 6)
				     : (SCHED_CAPACITY_SCALE * 3 / 4);

	if (rfx_c->iowait_boost >= max_cap)
		rfx_c->iowait_boost = min_t(unsigned int, rfx_c->iowait_boost << 1, cap);
	else
		rfx_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long rfx_iowait_apply(struct rfx_cpu *rfx_c, u64 time, unsigned long max_cap)
{
	if (!rfx_c->iowait_boost)
		return 0;
	if (rfx_iowait_reset(rfx_c, time, false))
		return 0;
	if (!rfx_c->iowait_boost_pending) {
		rfx_c->iowait_boost >>= 1;
		if (rfx_c->iowait_boost < IOWAIT_BOOST_MIN) {
			rfx_c->iowait_boost = 0;
			return 0;
		}
	}
	rfx_c->iowait_boost_pending = false;
	return (rfx_c->iowait_boost * max_cap) >> SCHED_CAPACITY_SHIFT;
}

static inline bool rfx_frame_active(struct rfx_policy *rfx_pol, u64 time)
{
	return rfx_pol->frame_boost_end_ns && time < rfx_pol->frame_boost_end_ns;
}

static void rfx_detect_render(struct rfx_policy *rfx_pol, unsigned int util_pct, u64 time)
{
	bool render_hint = atomic_read(&rfx_render_hint) != 0;

	if (render_hint) {
		rfx_pol->render_boost_end_ns = time + RENDER_BOOST_NS;
		return;
	}

	if (util_pct >= RENDER_BOOST_PCT) {
		if (!rfx_pol->render_detect_start_ns) {
			rfx_pol->render_detect_start_ns = time;
			rfx_pol->render_detect_count = 1;
		} else if (time - rfx_pol->render_detect_start_ns < RENDER_DETECT_NS) {
			rfx_pol->render_detect_count++;
			if (rfx_pol->render_detect_count >= RENDER_CONF_COUNT) {
				rfx_pol->render_boost_end_ns = time + RENDER_BOOST_NS;
				rfx_pol->render_detect_start_ns = 0;
				rfx_pol->render_detect_count = 0;
			}
		} else {
			rfx_pol->render_detect_start_ns = time;
			rfx_pol->render_detect_count = 1;
		}
	} else {
		rfx_pol->render_detect_start_ns = 0;
		rfx_pol->render_detect_count = 0;
	}
}

static unsigned int rfx_gaming_freq(struct rfx_policy *rfx_pol, unsigned long util,
				    unsigned long max_cap, bool render_boost, u64 time)
{
	struct cpufreq_policy *policy = rfx_pol->policy;
	bool little = rfx_is_little(max_cap);
	bool prime = rfx_is_prime(max_cap);
	unsigned int band_min, band_max, scale;
	unsigned int freq, tcap;
	unsigned int util_pct;
	unsigned int boost_pct;

	if (!policy || !max_cap)
		return 0;

	if (little) {
		band_min = GAMING_BAND_LITTLE_MIN;
		band_max = GAMING_BAND_LITTLE_MAX;
		scale = GAMING_SCALE_LITTLE;
	} else if (prime) {
		band_min = GAMING_BAND_PRIME_MIN;
		band_max = GAMING_BAND_PRIME_MAX;
		scale = GAMING_SCALE_PRIME;
	} else {
		band_min = GAMING_BAND_BIG_MIN;
		band_max = GAMING_BAND_BIG_MAX;
		scale = GAMING_SCALE_BIG;
	}

	band_min = rfx_abs(policy, band_min);
	band_max = rfx_abs(policy, band_max);

	if (band_max <= band_min)
		band_max = band_min + 100000;

	util_pct = max_cap ? (unsigned int)(util * 100 / max_cap) : 0;

	freq = band_min + (unsigned int)((u64)(band_max - band_min) * util_pct * scale / 100000);

	if (render_boost || (rfx_pol->render_boost_end_ns && time < rfx_pol->render_boost_end_ns)) {
		unsigned int boost = (band_max - band_min) * 15 / 100;
		freq = min(freq + boost, band_max);
	}

	boost_pct = rfx_pol->frame_boost_level;
	if (boost_pct > 0) {
		unsigned int boost = (band_max - band_min) * boost_pct / 100;
		freq = min(freq + boost, band_max);
	}

	if (rfx_pol->frame_boost_end_ns && time < rfx_pol->frame_boost_end_ns) {
		if (prime)
			freq = max(freq, rfx_floor_of_max(policy, FRAME_FLOOR_PRIME_PCT));
		else if (!little)
			freq = max(freq, rfx_floor_of_max(policy, FRAME_FLOOR_BIG_PCT));
	}

	if (!little) {
		unsigned int hi = band_min + (band_max - band_min) * 70 / 100;
		unsigned int lo = band_min + (band_max - band_min) * 60 / 100;

		if (freq >= hi)
			rfx_pol->sustain_hold_end_ns = time + SUSTAIN_BASE_NS;
		if (time < rfx_pol->sustain_hold_end_ns)
			freq = max(freq, lo);
	}

	tcap = rfx_thermal_cap(rfx_pol, time);
	if (tcap < 100)
		freq = min(freq, rfx_pct_of_max(policy, tcap));

	freq = min(freq, policy->max);
	freq = max(freq, band_min);

	return freq;
}

static unsigned int rfx_daily_freq(struct rfx_policy *rfx_pol, unsigned long util,
				   unsigned long max_cap, u64 time)
{
	struct cpufreq_policy *policy = rfx_pol->policy;
	bool little = rfx_is_little(max_cap);
	unsigned int freq, tcap;
	unsigned int ema_pct;

	if (!policy || !max_cap)
		return 0;

	ema_pct = rfx_ema_filter(rfx_pol,
				 max_cap ? (unsigned int)(util * 100 / max_cap) : 0);

	freq = (unsigned int)((u64)policy->cpuinfo.max_freq * ema_pct / 100);
	freq = clamp_t(unsigned int, freq, policy->cpuinfo.min_freq, policy->cpuinfo.max_freq);

	if (rfx_pol->in_deep_idle)
		freq = policy->cpuinfo.min_freq;
	else if (!little && rfx_pol->interactive_end_ns && time < rfx_pol->interactive_end_ns)
		freq = max(freq, rfx_floor_of_max(policy, INTERACTIVE_FLOOR_PCT));

	tcap = rfx_thermal_cap(rfx_pol, time);
	if (tcap < 100)
		freq = min(freq, rfx_pct_of_max(policy, tcap));

	if (!little) {
		u64 in_ts = atomic64_read(&rfx_input_ts_ns);
		if (in_ts && time - in_ts < INTERACT_WINDOW_NS)
			freq = max(freq, rfx_abs(policy, INTERACT_BIG_KHZ));
	} else {
		u64 in_ts = atomic64_read(&rfx_input_ts_ns);
		if (in_ts && time - in_ts < INTERACT_WINDOW_NS)
			freq = max(freq, rfx_abs(policy, INTERACT_LITTLE_KHZ));
	}

	freq = min(freq, policy->max);
	return freq;
}

static unsigned int rfx_calc_freq(struct rfx_policy *rfx_pol, unsigned long util,
				  unsigned long max_cap, bool is_heavy, u64 time)
{
	unsigned int freq;
	bool render_boost;

	render_boost = rfx_pol->render_boost_end_ns && time < rfx_pol->render_boost_end_ns;

	if (rfx_gaming()) {
		freq = rfx_gaming_freq(rfx_pol, util, max_cap, render_boost, time);
	} else {
		freq = rfx_daily_freq(rfx_pol, util, max_cap, time);
	}

	if (freq == rfx_pol->cached_raw_freq && !rfx_pol->need_freq_update)
		return rfx_pol->next_freq;

	rfx_pol->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(rfx_pol->policy, freq);
}

static bool rfx_should_update(struct rfx_policy *rfx_pol, u64 time)
{
	if (!rfx_pol || !rfx_pol->policy)
		return false;
	if (!cpufreq_this_cpu_can_update(rfx_pol->policy))
		return false;

	if (unlikely(READ_ONCE(rfx_pol->limits_changed))) {
		WRITE_ONCE(rfx_pol->limits_changed, false);
		rfx_pol->need_freq_update = true;
		smp_mb();
		return true;
	}
	if (rfx_pol->need_freq_update)
		return true;

	if (rfx_gaming())
		return true;

	if (rfx_pol->in_deep_idle)
		return (time - rfx_pol->last_downfreq_time) >= (2 * NSEC_PER_USEC);

	return (time - rfx_pol->last_upfreq_time) >= rfx_pol->freq_update_delay_ns ||
	       (time - rfx_pol->last_downfreq_time) >= rfx_pol->down_rate_delay_ns;
}

static bool rfx_set_next_freq(struct rfx_policy *rfx_pol, u64 time,
			      unsigned int next_freq, bool force_down)
{
	if (!rfx_pol)
		return false;

	if (rfx_pol->need_freq_update) {
		rfx_pol->need_freq_update = false;
		if (rfx_pol->next_freq == next_freq)
			return false;
	} else if (rfx_pol->next_freq == next_freq) {
		return false;
	}

	if (next_freq < rfx_pol->next_freq) {
		if (!force_down && !rfx_gaming()) {
			s64 dd = time - rfx_pol->last_downfreq_time;
			if (rfx_pol->down_rate_delay_ns > 0 && dd < rfx_pol->down_rate_delay_ns)
				return false;
		}
		rfx_pol->last_downfreq_time = time;
	} else {
		s64 ud = time - rfx_pol->last_upfreq_time;
		if (!rfx_gaming() && rfx_pol->up_rate_delay_ns > 0 && ud < rfx_pol->up_rate_delay_ns)
			return false;
		rfx_pol->last_upfreq_time = time;
	}

	rfx_pol->next_freq = next_freq;
	return true;
}

#ifdef CONFIG_NO_HZ_COMMON
static bool rfx_nohz_idle(struct rfx_cpu *rfx_c)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(rfx_c->cpu);
	bool increased = idle_calls != rfx_c->saved_idle_calls;

	rfx_c->saved_idle_calls = idle_calls;
	return increased;
}
#else
static inline bool rfx_nohz_idle(struct rfx_cpu *rfx_c) { return false; }
#endif

static void rfx_clear_expired(struct rfx_policy *rfx_pol, u64 time)
{
	if (rfx_pol->interactive_end_ns && time >= rfx_pol->interactive_end_ns)
		rfx_pol->interactive_end_ns = 0;
	if (rfx_pol->frame_boost_end_ns && time >= rfx_pol->frame_boost_end_ns)
		rfx_pol->frame_boost_end_ns = 0;
	if (rfx_pol->gaming_lock_end_ns && time >= rfx_pol->gaming_lock_end_ns &&
	    !rfx_gaming())
		rfx_pol->gaming_lock_end_ns = 0;
	if (rfx_pol->render_boost_end_ns && time >= rfx_pol->render_boost_end_ns)
		rfx_pol->render_boost_end_ns = 0;
}

static unsigned int rfx_decide(struct rfx_cpu *lead, u64 time, unsigned long pooled_util,
			       unsigned int util_pct, bool *force_down)
{
	struct rfx_policy *rfx_pol = lead->rfx_policy;
	unsigned long max_cap = arch_scale_cpu_capacity(lead->cpu);
	unsigned int eff_pct, next_f;
	unsigned long effective;
	bool is_heavy;

	if (rfx_gaming()) {
		rfx_detect_render(rfx_pol, util_pct, time);
		eff_pct = util_pct;
	} else {
		eff_pct = rfx_ema_filter(rfx_pol, util_pct);
	}

	effective = max_cap * eff_pct / 100;
	if (effective < pooled_util)
		effective = pooled_util;

	if (!rfx_gaming()) {
		bool nohz_idle = rfx_nohz_idle(lead);

		if (nohz_idle && rfx_pol->last_real_update_ns &&
		    (time - rfx_pol->last_real_update_ns) > IDLE_STALE_NS &&
		    util_pct < IDLE_ENTER_PCT)
			rfx_pol->in_deep_idle = true;
		else if (util_pct >= IDLE_ENTER_PCT) {
			rfx_pol->in_deep_idle = false;
			rfx_pol->last_real_update_ns = time;
		}
	} else if (util_pct >= IDLE_ENTER_PCT) {
		rfx_pol->last_real_update_ns = time;
	}

	if (rfx_gaming()) {
		rfx_pol->in_deep_idle = false;
		if (util_pct >= 35)
			rfx_pol->in_heavy_mode = true;
		else if (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns)
			rfx_pol->in_heavy_mode = true;
		else if (!rfx_pol->gaming_lock_end_ns ||
			 (time - rfx_pol->gaming_lock_end_ns) > GAMING_HYSTERESIS_NS)
			rfx_pol->in_heavy_mode = false;

		if (rfx_pol->in_heavy_mode)
			rfx_pol->gaming_lock_end_ns = time + GAMING_LOCK_NS;
	} else {
		if (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns)
			goto daily_out;
		rfx_pol->gaming_lock_end_ns = 0;
		rfx_pol->in_heavy_mode = false;
	}

	rfx_frame_observe(rfx_pol, time);
	daily_out:

	if (!rfx_gaming() && util_pct >= 1) {
		rfx_pol->interactive_end_ns = time + INTERACTIVE_DUR_NS;
		rfx_pol->in_deep_idle = false;
	}

	is_heavy = rfx_pol->in_heavy_mode ||
		   (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns);

	next_f = rfx_calc_freq(rfx_pol, effective, max_cap, is_heavy, time);

	if (is_heavy && lead->prev_util_pct > SUSTAIN_DROP_THR &&
	    lead->prev_util_pct > util_pct &&
	    (lead->prev_util_pct - util_pct) >= SUSTAIN_DROP_THR &&
	    (!rfx_pol->sustain_hold_end_ns || time > rfx_pol->sustain_hold_end_ns))
		rfx_pol->sustain_hold_end_ns = time + SUSTAIN_FRAME_NS;

	lead->prev_util_pct2 = lead->prev_util_pct;
	lead->prev_util_pct = util_pct;

	*force_down = !rfx_gaming() && rfx_pol->in_deep_idle && !is_heavy;
	return next_f;
}

static void rfx_commit(struct rfx_policy *rfx_pol, u64 time, unsigned int next_f, bool force_down)
{
	if (!rfx_set_next_freq(rfx_pol, time, next_f, force_down))
		return;

	if (rfx_pol->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(rfx_pol->policy, rfx_pol->next_freq);
	} else if (!rfx_pol->work_in_progress) {
		rfx_pol->work_in_progress = true;
		irq_work_queue(&rfx_pol->irq_work);
	}
}

static void rfx_update_single(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	unsigned long max_cap, boost, util;
	unsigned int util_pct, next_f;
	bool force_down;

	rfx_clear_expired(rfx_pol, time);
	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (!rfx_should_update(rfx_pol, time))
		return;

	max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	boost = rfx_iowait_apply(rfx_c, time, max_cap);
	rfx_get_util_gki510(rfx_c->cpu, boost, &rfx_c->util, &rfx_c->bwmin);
	util = max(rfx_c->util, boost);
	util_pct = max_cap ? (unsigned int)(util * 100 / max_cap) : 0;

	next_f = rfx_decide(rfx_c, time, util, util_pct, &force_down);
	rfx_commit(rfx_pol, time, next_f, force_down);
}

static void rfx_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = rfx_pol->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	unsigned long pooled = 0;
	unsigned int j, max_pct = 0, next_f;
	struct rfx_cpu *lead = NULL;
	bool force_down;

	rfx_clear_expired(rfx_pol, time);

	raw_spin_lock(&rfx_pol->update_lock);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (!rfx_should_update(rfx_pol, time)) {
		raw_spin_unlock(&rfx_pol->update_lock);
		return;
	}

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *j_c = per_cpu_ptr(&rfx_cpu_table, j);
		unsigned long j_boost = rfx_iowait_apply(j_c, time, max_cap);
		unsigned long j_util;
		unsigned int j_pct;

		rfx_get_util_gki510(j_c->cpu, j_boost, &j_c->util, &j_c->bwmin);
		j_util = max(j_c->util, j_boost);
		j_pct = max_cap ? (unsigned int)(j_util * 100 / max_cap) : 0;

		if (j == cpumask_first(policy->cpus))
			lead = j_c;
		if (j_util > pooled)
			pooled = j_util;
		if (j_pct > max_pct)
			max_pct = j_pct;
	}

	if (lead) {
		next_f = rfx_decide(lead, time, pooled, max_pct, &force_down);
		rfx_commit(rfx_pol, time, next_f, force_down);
	}

	raw_spin_unlock(&rfx_pol->update_lock);
}

static void rfx_work(struct kthread_work *work)
{
	struct rfx_policy *rfx_pol = container_of(work, struct rfx_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&rfx_pol->update_lock, flags);
	freq = rfx_pol->next_freq;
	rfx_pol->work_in_progress = false;
	raw_spin_unlock_irqrestore(&rfx_pol->update_lock, flags);

	mutex_lock(&rfx_pol->work_lock);
	cpufreq_driver_target(rfx_pol->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&rfx_pol->work_lock);
}

static void rfx_irq_work(struct irq_work *irq_work)
{
	struct rfx_policy *rfx_pol = container_of(irq_work, struct rfx_policy, irq_work);

	kthread_queue_work(&rfx_pol->worker, &rfx_pol->work);
}

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->rate_limit_us);
}
static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	struct rfx_policy *pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->rate_limit_us = val;
	list_for_each_entry(pol, &attr_set->policy_list, tunables_hook)
		pol->freq_update_delay_ns = (s64)val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr rate_limit_us = __ATTR_RW(rate_limit_us);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->up_rate_limit_us);
}
static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	struct rfx_policy *pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->up_rate_limit_us = val;
	list_for_each_entry(pol, &attr_set->policy_list, tunables_hook)
		pol->up_rate_delay_ns = (s64)val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->down_rate_limit_us);
}
static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	struct rfx_policy *pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->down_rate_limit_us = val;
	list_for_each_entry(pol, &attr_set->policy_list, tunables_hook)
		pol->down_rate_delay_ns = (s64)val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&rfx_gaming_mode));
}
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct rfx_policy *pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;

	atomic_set(&rfx_gaming_mode, val);
	WRITE_ONCE(sched_gaming_active, val);

	if (val) {
		atomic_set(&rfx_render_hint, 1);
	} else {
		unsigned long flags;
		spin_lock_irqsave(&rfx_frame_hist_lock, flags);
		memset(rfx_frame_hist, 0, sizeof(rfx_frame_hist));
		rfx_frame_hist_idx = 0;
		spin_unlock_irqrestore(&rfx_frame_hist_lock, flags);

		atomic_set(&rfx_render_hint, 0);
		list_for_each_entry(pol, &attr_set->policy_list, tunables_hook) {
			pol->gaming_lock_end_ns = 0;
			pol->frame_boost_end_ns = 0;
			pol->sustain_hold_end_ns = 0;
			pol->render_boost_end_ns = 0;
			pol->in_heavy_mode = false;
			pol->spike_confirm = 0;
			pol->frame_boost_level = 0;
			pol->need_freq_update = true;
		}
	}
	return count;
}
static struct governor_attr gaming_mode = __ATTR_RW(gaming_mode);

static ssize_t thermal_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&rfx_thermal_cap_pct));
}
static ssize_t thermal_cap_pct_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val < 30 || val > 100)
		return -EINVAL;
	atomic_set(&rfx_thermal_cap_pct, val);
	return count;
}
static struct governor_attr thermal_cap_pct = __ATTR_RW(thermal_cap_pct);

static ssize_t frame_time_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&rfx_frame_time_us));
}
static ssize_t frame_time_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&rfx_frame_time_us, val);
	return count;
}
static struct governor_attr frame_time_us = __ATTR_RW(frame_time_us);

static ssize_t frame_budget_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&rfx_frame_budget_us));
}
static ssize_t frame_budget_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val < 1000 || val > 100000)
		return -EINVAL;
	atomic_set(&rfx_frame_budget_us, val);
	return count;
}
static struct governor_attr frame_budget_us = __ATTR_RW(frame_budget_us);

static ssize_t render_hint_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&rfx_render_hint));
}
static ssize_t render_hint_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	atomic_set(&rfx_render_hint, val);
	return count;
}
static struct governor_attr render_hint = __ATTR_RW(render_hint);

static struct attribute *rfx_little_attrs[] = {
	&rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_little);

static struct attribute *rfx_big_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_big);

static struct attribute *rfx_prime_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&gaming_mode.attr,
	&thermal_cap_pct.attr,
	&frame_time_us.attr,
	&frame_budget_us.attr,
	&render_hint.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_prime);

static void rfx_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);

	kfree(to_rfx_tunables(attr_set));
}

static struct kobj_type rfx_little_ktype = {
	.default_groups = rfx_little_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};
static struct kobj_type rfx_big_ktype = {
	.default_groups = rfx_big_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};
static struct kobj_type rfx_prime_ktype = {
	.default_groups = rfx_prime_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};

static struct cpufreq_governor vorpal_gov;

static struct rfx_policy *rfx_policy_alloc(struct cpufreq_policy *policy)
{
	struct rfx_policy *pol = kzalloc(sizeof(*pol), GFP_KERNEL);

	if (!pol)
		return NULL;
	pol->policy = policy;
	raw_spin_lock_init(&pol->update_lock);
	return pol;
}

static int rfx_kthread_create(struct rfx_policy *rfx_pol)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	struct cpufreq_policy *policy = rfx_pol->policy;
	struct task_struct *thread;
	int ret;

	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&rfx_pol->work, rfx_work);
	kthread_init_worker(&rfx_pol->worker);
	thread = kthread_create(kthread_worker_fn, &rfx_pol->worker,
				"rfx_gov/%d", cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("vorpal: kthread create failed %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("vorpal: failed to set SCHED_FIFO\n");
		return ret;
	}

	rfx_pol->thread = thread;
	if (policy->dvfs_possible_from_any_cpu)
		set_cpus_allowed_ptr(thread, policy->related_cpus);
	else
		kthread_bind_mask(thread, policy->related_cpus);

	init_irq_work(&rfx_pol->irq_work, rfx_irq_work);
	mutex_init(&rfx_pol->work_lock);
	wake_up_process(thread);
	return 0;
}

static void rfx_kthread_stop(struct rfx_policy *rfx_pol)
{
	if (rfx_pol->policy->fast_switch_enabled)
		return;
	kthread_flush_worker(&rfx_pol->worker);
	kthread_stop(rfx_pol->thread);
	mutex_destroy(&rfx_pol->work_lock);
}

static struct rfx_tunables *rfx_tunables_alloc(struct rfx_policy *rfx_pol)
{
	struct rfx_tunables *t = kzalloc(sizeof(*t), GFP_KERNEL);

	if (t) {
		gov_attr_set_init(&t->attr_set, &rfx_pol->tunables_hook);
		if (!have_governor_per_policy())
			rfx_global_tunables = t;
	}
	return t;
}

static void rfx_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		rfx_global_tunables = NULL;
}

static int rfx_init(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol;
	struct rfx_tunables *t;
	struct kobj_type *ktype;
	unsigned long max_cap;
	int ret;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	rfx_pol = rfx_policy_alloc(policy);
	if (!rfx_pol) {
		ret = -ENOMEM;
		goto disable_fast;
	}

	ret = rfx_kthread_create(rfx_pol);
	if (ret)
		goto free_pol;

	mutex_lock(&rfx_global_tunables_lock);

	if (rfx_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = rfx_pol;
		rfx_pol->tunables = rfx_global_tunables;
		gov_attr_set_get(&rfx_global_tunables->attr_set, &rfx_pol->tunables_hook);
		rfx_pol->freq_update_delay_ns = (s64)rfx_global_tunables->rate_limit_us * NSEC_PER_USEC;
		rfx_pol->up_rate_delay_ns = (s64)rfx_global_tunables->up_rate_limit_us * NSEC_PER_USEC;
		rfx_pol->down_rate_delay_ns = (s64)rfx_global_tunables->down_rate_limit_us * NSEC_PER_USEC;
		goto out;
	}

	t = rfx_tunables_alloc(rfx_pol);
	if (!t) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	if (rfx_is_little(max_cap)) {
		t->cluster_type = CLUSTER_LITTLE;
		t->rate_limit_us = LITTLE_UP_US;
		t->up_rate_limit_us = LITTLE_UP_US;
		t->down_rate_limit_us = LITTLE_DOWN_US;
		ktype = &rfx_little_ktype;
	} else if (rfx_is_prime(max_cap)) {
		t->cluster_type = CLUSTER_PRIME;
		t->rate_limit_us = 1;
		t->up_rate_limit_us = PRIME_UP_US;
		t->down_rate_limit_us = PRIME_DOWN_US;
		ktype = &rfx_prime_ktype;
	} else {
		t->cluster_type = CLUSTER_BIG;
		t->rate_limit_us = BIG_UP_US;
		t->up_rate_limit_us = BIG_UP_US;
		t->down_rate_limit_us = BIG_DOWN_US;
		ktype = &rfx_big_ktype;
	}

	policy->governor_data = rfx_pol;
	rfx_pol->tunables = t;
	rfx_pol->freq_update_delay_ns = (s64)t->rate_limit_us * NSEC_PER_USEC;
	rfx_pol->up_rate_delay_ns = (s64)t->up_rate_limit_us * NSEC_PER_USEC;
	rfx_pol->down_rate_delay_ns = (s64)t->down_rate_limit_us * NSEC_PER_USEC;

	ret = kobject_init_and_add(&t->attr_set.kobj, ktype,
				   get_governor_parent_kobj(policy), "%s", vorpal_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&rfx_global_tunables_lock);
	return 0;

fail:
	kobject_put(&t->attr_set.kobj);
	policy->governor_data = NULL;
	rfx_clear_global_tunables();
stop_kthread:
	rfx_kthread_stop(rfx_pol);
	mutex_unlock(&rfx_global_tunables_lock);
free_pol:
	kfree(rfx_pol);
disable_fast:
	cpufreq_disable_fast_switch(policy);
	pr_err("vorpal: init failed %d\n", ret);
	return ret;
}

static void rfx_exit(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	struct rfx_tunables *t = rfx_pol->tunables;
	unsigned int count;

	mutex_lock(&rfx_global_tunables_lock);
	count = gov_attr_set_put(&t->attr_set, &rfx_pol->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		rfx_clear_global_tunables();
	mutex_unlock(&rfx_global_tunables_lock);

	rfx_kthread_stop(rfx_pol);
	kfree(rfx_pol);
	cpufreq_disable_fast_switch(policy);
}

static int rfx_start(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	void (*uu)(struct update_util_data *, u64, unsigned int);
	unsigned int cpu;
	u64 now = ktime_get_ns();

	rfx_pol->freq_update_delay_ns = (s64)rfx_pol->tunables->rate_limit_us * NSEC_PER_USEC;
	rfx_pol->up_rate_delay_ns = (s64)rfx_pol->tunables->up_rate_limit_us * NSEC_PER_USEC;
	rfx_pol->down_rate_delay_ns = (s64)rfx_pol->tunables->down_rate_limit_us * NSEC_PER_USEC;

	rfx_pol->last_upfreq_time = now;
	rfx_pol->last_downfreq_time = now;
	rfx_pol->next_freq = policy->cur > 0 ? policy->cur : policy->cpuinfo.min_freq;
	rfx_pol->work_in_progress = false;
	rfx_pol->limits_changed = false;
	rfx_pol->cached_raw_freq = 0;
	rfx_pol->need_freq_update = false;
	rfx_pol->gaming_lock_end_ns = 0;
	rfx_pol->frame_boost_end_ns = 0;
	rfx_pol->sustain_hold_end_ns = 0;
	rfx_pol->interactive_end_ns = 0;
	rfx_pol->render_boost_end_ns = 0;
	rfx_pol->thermal_cap_smoothed = atomic_read(&rfx_thermal_cap_pct);
	rfx_pol->thermal_step_ns = now;
	rfx_pol->in_deep_idle = false;
	rfx_pol->last_real_update_ns = now;
	rfx_pol->in_heavy_mode = false;
	rfx_pol->ema_util_pct = 0;
	rfx_pol->spike_confirm = 0;
	rfx_pol->render_detect_count = 0;
	rfx_pol->render_detect_start_ns = 0;
	rfx_pol->frame_boost_level = 0;
	rfx_pol->frame_boost_applied_ns = 0;

	uu = policy_is_shared(policy) ? rfx_update_shared : rfx_update_single;

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = per_cpu_ptr(&rfx_cpu_table, cpu);

		memset(rfx_c, 0, sizeof(*rfx_c));
		rfx_c->cpu = cpu;
		rfx_c->rfx_policy = rfx_pol;
		cpufreq_add_update_util_hook(cpu, &rfx_c->update_util, uu);
	}
	return 0;
}

static void rfx_stop(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&rfx_pol->irq_work);
		kthread_cancel_work_sync(&rfx_pol->work);
	}
}

static void rfx_limits(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&rfx_pol->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&rfx_pol->work_lock);
	}
	smp_wmb();
	WRITE_ONCE(rfx_pol->limits_changed, true);
}

static struct cpufreq_governor vorpal_gov = {
	.name	= "vorpal",
	.owner	= THIS_MODULE,
	.flags	= CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init	= rfx_init,
	.exit	= rfx_exit,
	.start	= rfx_start,
	.stop	= rfx_stop,
	.limits	= rfx_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_VORPAL
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &vorpal_gov;
}
#endif

static void rfx_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	if (type == EV_ABS || type == EV_KEY)
		atomic64_set(&rfx_input_ts_ns, ktime_get_ns());
}

static int rfx_input_connect(struct input_handler *handler, struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "vorpal";

	error = input_register_handle(handle);
	if (error)
		goto err_free;
	error = input_open_device(handle);
	if (error)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return error;
}

static void rfx_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id rfx_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] = BIT_MASK(ABS_MT_POSITION_X) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler rfx_input_handler = {
	.event		= rfx_input_event,
	.connect	= rfx_input_connect,
	.disconnect	= rfx_input_disconnect,
	.name		= "vorpal_boost",
	.id_table	= rfx_input_ids,
};

static int __init vorpal_init(void)
{
	int ret;

	pr_info("Vorpal Governor v%s by %s\n", VORPAL_VER, VORPAL_AUTH);
	ret = cpufreq_register_governor(&vorpal_gov);
	if (ret)
		return ret;

	if (input_register_handler(&rfx_input_handler))
		pr_warn("vorpal: input handler failed; interaction boost off\n");
	return 0;
}

static void __exit vorpal_exit(void)
{
	input_unregister_handler(&rfx_input_handler);
	cpufreq_unregister_governor(&vorpal_gov);
}

module_init(vorpal_init);
module_exit(vorpal_exit);

MODULE_AUTHOR(VORPAL_AUTH);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v7.0 - Gaming & Efficiency");
