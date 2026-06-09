// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v4.0 - Gaming & Efficiency Edition
 *
 * schedutil-derived, fast-switch capable governor for GKI 5.10 big.LITTLE.
 *
 * Honest scope (what the C can and cannot do):
 *   - This governor only selects CPU frequencies. It CANNOT by itself "lock"
 *     a frame rate, bound SoC power to a wattage, or hold a junction/skin
 *     temperature. Those are properties of the full DVFS stack (OPP tables,
 *     the thermal framework / cooling devices, the GPU governor and the
 *     display/compositor). What this file does:
 *       * gaming_mode=1: hold per-cluster frequency FLOORS so render/sim
 *         threads never starve, while CAPPING below f_max to leave thermal
 *         headroom; step the cap down predictively when sustained high
 *         frequency is observed, and honour an externally-published thermal
 *         cap (see rfx_thermal_cap_pct, fed by a slow-path poller in
 *         userspace or a delayed_work, NOT read in the atomic hook).
 *       * gaming_mode=0: collapse to the lowest viable frequency fast, park
 *         little cores at f_min on idle, and damp churn for battery.
 *   - Frame pacing: userspace (the vorpal frame feeder) publishes the last
 *     observed frame time and the target budget via sysfs; when a frame
 *     overruns its budget we raise the cluster floor for one boost window.
 *     This is the only "frame aware" input - the kernel never reads the
 *     display pipeline directly (it would sleep / break KMI).
 *
 * ABI: this is a module; the util getter and DL-bandwidth check are owned and
 * EXPORT_SYMBOL_GPL'd by core sched (cpufreq_schedutil.c) and consumed here.
 * Editing the RFX_* default values below is KMI-safe; nothing here changes a
 * GKI-exported struct layout.
 *
 * Author: Templar Dev (Steambot12)
 */

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
#include <uapi/linux/sched/types.h>	/* struct sched_attr */
#include <linux/timekeeping.h>
#include <linux/ktime.h>
#include <linux/atomic.h>
#include <linux/tick.h>
#include <linux/jiffies.h>
#include <linux/irq_work.h>

/* Owned + EXPORT_SYMBOL_GPL'd in kernel/sched/cpufreq_schedutil.c. */
extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);

/* Owned + EXPORT_SYMBOL_GPL'd in kernel/sched/fair.c; the load balancer reads
 * it to bias toward cache stickiness while gaming (fewer migrations). */
extern int sched_gaming_active;

#define CPUFREQ_VORPAL_VERSION  "4.0"
#define CPUFREQ_VORPAL_AUTHOR   "Templar Dev"

/* ===================== Tunable defaults (KMI-safe) ===================== */

/* Cluster classification by arch capacity (1024 == biggest core). */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* Rate limiting (us). up==0 means "raise immediately"; down is damped to
 * suppress the high-frequency oscillation that shows up as 80%+ load spikes. */
#define RFX_LITTLE_UP_US		0
#define RFX_LITTLE_DOWN_US		4000
#define RFX_BIG_UP_US			0
#define RFX_BIG_DOWN_US			8000
#define RFX_PRIME_UP_US			0
#define RFX_PRIME_DOWN_US		8000
#define RFX_GAMING_DOWN_DELAY_US	35000

/* Gaming-mode per-cluster floors / caps, as percent of policy f_max.
 * Floors keep render/sim threads off the wall; caps leave thermal headroom
 * (caps are deliberately < 100 so we sustain rather than spike-and-throttle). */
#define RFX_GAMING_FLOOR_PRIME_PCT	88
#define RFX_GAMING_FLOOR_BIG_PCT	78
#define RFX_GAMING_CAP_PRIME_PCT	99
#define RFX_GAMING_CAP_BIG_PCT		94
#define RFX_GAMING_FLOOR_LITTLE_PCT	45
#define RFX_GAMING_LITTLE_CAP_PCT	85
#define RFX_GAMING_HEADROOM_PCT		28
#define RFX_GAMING_LOCK_NS		(800  * NSEC_PER_MSEC)
#define RFX_GAMING_HYSTERESIS_NS	(6000 * NSEC_PER_MSEC)

/* Gaming spike rejection. A lobby/waiting-room asset load is a short burst that
 * rockets util to 80%+ for a few ticks without any sustained 120fps render
 * demand behind it. We only treat a high load as "real heavy" once it persists
 * for SPIKE_CONFIRM consecutive updates; until then the burst is held to a
 * comfort cap so it cannot wake every core to f_max. Frame-pacing (userspace
 * frame_time feed) is exempt - a genuine 120fps frame overrun bypasses this. */
#define RFX_GAMING_HEAVY_ENTER_PCT	60
#define RFX_GAMING_SPIKE_CONFIRM	4
#define RFX_GAMING_BURST_CAP_PCT	60

/* Predictive thermal step-down (frequency-based, governor-internal).
 *
 * NOTE: this heuristic throttles purely on *frequency residency*, not measured
 * temperature. On a Prime core that legitimately sustains ~96% of f_max during
 * heavy gameplay it tripped every SETTLE_NS, producing the periodic micro-drops
 * (~96% -> SOFT_CAP) that read as recurring sub-100fps dips. The real thermal
 * ceiling is the *measured* cap published via rfx_thermal_cap_pct (fed by the
 * thermal framework / userspace daemon) and is left fully intact. HIGH_PCT is
 * therefore set to 100 so the freq-only path never self-throttles a healthy
 * sustained clock; if it ever does trip, SOFT_CAP keeps the step shallow. */
#define RFX_THERMAL_HIGH_PCT		100
#define RFX_THERMAL_LOW_PCT		70
#define RFX_THERMAL_SOFT_CAP_PCT	96
#define RFX_THERMAL_SETTLE_NS		(4000 * NSEC_PER_MSEC)
#define RFX_THERMAL_THROTTLE_NS		(4000 * NSEC_PER_MSEC)

/* Frame pacing. Budget defaults to a 120 fps frame (8333 us). When userspace
 * reports a frame time past BUDGET * MISS_NUM/MISS_DEN we raise the cluster
 * floor for one boost window. Zero frame_time_us == feature idle. */
#define RFX_FRAME_BUDGET_US_DEFAULT	8333
#define RFX_FRAME_MISS_NUM		9
#define RFX_FRAME_MISS_DEN		10
#define RFX_FRAME_BOOST_NS		(120 * NSEC_PER_MSEC)
#define RFX_FRAME_FLOOR_PRIME_PCT	90
#define RFX_FRAME_FLOOR_BIG_PCT		85

/* Input boost (userspace pulses this on touch). */
#define RFX_INPUT_BOOST_FREQ_PCT	95

/* Idle / deep-idle parking (daily mode). */
#define RFX_IDLE_STALE_NS		(30 * NSEC_PER_MSEC)
#define RFX_IDLE_ENTER_PCT		3

/* Daily interactive floor so UI scroll never starts cold. */
#define RFX_INTERACTIVE_DURATION_NS	(3000 * NSEC_PER_MSEC)
#define RFX_INTERACTIVE_FLOOR_BIG_PCT	15

/* EMA load tracking. Asymmetric alpha (out of 256): fast attack so we never
 * lag a real ramp, slow decay so transient dips don't drop frequency under a
 * steady render load - this is the core anti-jank / anti-spike signal. */
#define RFX_EMA_FAST_ALPHA		180
#define RFX_EMA_SLOW_ALPHA		25

/* Hold frequency briefly after a sharp util drop while heavy, so a single
 * idle frame in a render loop doesn't yank the clock down and cause a stall. */
#define RFX_BURST_DROP_THRESHOLD	12
#define RFX_SUSTAIN_HOLD_NS		(200 * NSEC_PER_MSEC)

#define SCHED_FLAGS_UGOV		0x10000000
#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

/* ===================== Types ===================== */

enum rfx_cluster_type {
	RFX_CLUSTER_LITTLE = 0,
	RFX_CLUSTER_BIG    = 1,
	RFX_CLUSTER_PRIME  = 2,
};

struct rfx_policy;

struct rfx_tunables {
	struct gov_attr_set attr_set;
	unsigned int rate_limit_us;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
	unsigned int input_boost_ms;
	unsigned int input_boost_pct;
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
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
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

	/* Boost / hold windows (ns deadlines). */
	u64 gaming_lock_end_ns;
	u64 input_boost_end_ns;
	unsigned int input_boost_freq;
	u64 frame_boost_end_ns;
	u64 sustain_hold_end_ns;
	u64 interactive_end_ns;

	/* Predictive thermal step-down. */
	u64 thermal_high_start_ns;
	u64 thermal_throttle_end_ns;
	bool thermal_throttle_active;

	/* Idle. */
	bool in_deep_idle;
	u64 last_real_update_ns;

	/* Heavy/gaming state. */
	bool in_heavy_mode;

	/* EMA. */
	unsigned int ema_util_pct;

	/* Gaming spike rejection (lobby asset-load bursts). */
	unsigned int spike_confirm;
	bool burst_capped;
};

static DEFINE_PER_CPU(struct rfx_cpu, rfx_cpu_table);

static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

/* ===================== Global signals (atomics) =====================
 *
 * All written from slow paths (sysfs / userspace feeder) and read lock-free
 * from the atomic util hook. Never read a sleeping source in the fast path.
 */

/* gaming_mode: 1 == gaming bias, 0 == battery bias. */
static atomic_t rfx_gaming_mode_global = ATOMIC_INIT(0);

/* Measured thermal cap as percent of f_max (100 == no cap). Published by the
 * slow-path thermal poller; the fast path only does an atomic_read. */
static atomic_t rfx_thermal_cap_pct = ATOMIC_INIT(100);

/* Frame pacing, fed by userspace. */
static atomic_t rfx_frame_time_us  = ATOMIC_INIT(0);
static atomic_t rfx_frame_budget_us = ATOMIC_INIT(RFX_FRAME_BUDGET_US_DEFAULT);

static inline bool rfx_gaming(void)
{
	return atomic_read(&rfx_gaming_mode_global) != 0;
}

/* ===================== Small helpers ===================== */

static inline bool rfx_is_little(unsigned long cap)
{
	return cap <= RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool rfx_is_prime(unsigned long cap)
{
	return cap >= RFX_PRIME_CAP_THRESHOLD;
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

static inline struct rfx_tunables *to_rfx_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct rfx_tunables, attr_set);
}

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bwmin))
		rfx_c->rfx_policy->need_freq_update = true;
}

/* ===================== EMA load filter ===================== */

static unsigned int rfx_ema_filter(struct rfx_policy *rfx_pol, unsigned int raw_pct)
{
	unsigned int ema = rfx_pol->ema_util_pct;
	unsigned int alpha = (raw_pct > ema) ? RFX_EMA_FAST_ALPHA : RFX_EMA_SLOW_ALPHA;

	ema = (ema * (256 - alpha) + raw_pct * alpha) / 256;

	/* Anti-lag clamp: on a steep genuine ramp don't trail the raw signal by
	 * more than ~15 points, otherwise the first busy frames render cold. */
	if (ema < raw_pct && raw_pct - ema > 30)
		ema = raw_pct - 15;

	rfx_pol->ema_util_pct = ema;
	return ema;
}

/* ===================== Predictive thermal step-down ===================== */

static void rfx_thermal_observe(struct rfx_policy *rfx_pol, unsigned int freq, u64 time)
{
	struct cpufreq_policy *policy = rfx_pol->policy;
	unsigned int freq_pct;

	if (!policy || !policy->cpuinfo.max_freq)
		return;

	if (!rfx_gaming()) {
		rfx_pol->thermal_throttle_active = false;
		rfx_pol->thermal_high_start_ns = 0;
		return;
	}

	freq_pct = freq * 100 / policy->cpuinfo.max_freq;

	if (freq_pct > RFX_THERMAL_HIGH_PCT) {
		if (!rfx_pol->thermal_high_start_ns) {
			rfx_pol->thermal_high_start_ns = time;
		} else if (time - rfx_pol->thermal_high_start_ns > RFX_THERMAL_SETTLE_NS) {
			rfx_pol->thermal_throttle_active = true;
			rfx_pol->thermal_throttle_end_ns = time + RFX_THERMAL_THROTTLE_NS;
		}
	} else if (freq_pct < RFX_THERMAL_LOW_PCT) {
		rfx_pol->thermal_high_start_ns = 0;
		if (rfx_pol->thermal_throttle_active && time > rfx_pol->thermal_throttle_end_ns)
			rfx_pol->thermal_throttle_active = false;
	}
}

/* Returns the effective thermal cap percent: min(predictive, measured). */
static unsigned int rfx_thermal_cap(struct rfx_policy *rfx_pol, u64 time)
{
	unsigned int cap = atomic_read(&rfx_thermal_cap_pct);

	if (cap > 100)
		cap = 100;

	if (rfx_pol->thermal_throttle_active) {
		if (time < rfx_pol->thermal_throttle_end_ns)
			cap = min(cap, (unsigned int)RFX_THERMAL_SOFT_CAP_PCT);
		else
			rfx_pol->thermal_throttle_active = false;
	}
	return cap;
}

/* ===================== Frame pacing ===================== */

/* If userspace reports a frame overrun, arm a short floor-boost window. */
static void rfx_frame_observe(struct rfx_policy *rfx_pol, u64 time)
{
	unsigned int ft = atomic_read(&rfx_frame_time_us);
	unsigned int budget = atomic_read(&rfx_frame_budget_us);

	if (!rfx_gaming() || !ft || !budget)
		return;

	if ((u64)ft * RFX_FRAME_MISS_DEN > (u64)budget * RFX_FRAME_MISS_NUM)
		rfx_pol->frame_boost_end_ns = time + RFX_FRAME_BOOST_NS;
}

/* ===================== iowait boost ===================== */

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

/* ===================== Heavy/gaming state ===================== */

static inline bool rfx_frame_active(struct rfx_policy *rfx_pol, u64 time)
{
	return rfx_pol->frame_boost_end_ns && time < rfx_pol->frame_boost_end_ns;
}

/* Returns true once a high load has persisted long enough to be a real render
 * load rather than a transient lobby burst. Also publishes burst_capped, which
 * tells rfx_calc_freq to hold an unconfirmed burst to a comfort cap. A genuine
 * frame overrun (userspace frame-pacing feed) is always treated as confirmed. */
static bool rfx_spike_confirmed(struct rfx_policy *rfx_pol, unsigned int util_pct, u64 time)
{
	if (rfx_frame_active(rfx_pol, time)) {
		rfx_pol->spike_confirm = RFX_GAMING_SPIKE_CONFIRM;
		rfx_pol->burst_capped = false;
		return true;
	}

	if (util_pct >= RFX_GAMING_HEAVY_ENTER_PCT) {
		if (rfx_pol->spike_confirm < RFX_GAMING_SPIKE_CONFIRM)
			rfx_pol->spike_confirm++;
	} else if (util_pct < RFX_GAMING_BURST_CAP_PCT) {
		rfx_pol->spike_confirm = 0;
	}

	rfx_pol->burst_capped = (util_pct >= RFX_GAMING_HEAVY_ENTER_PCT) &&
				(rfx_pol->spike_confirm < RFX_GAMING_SPIKE_CONFIRM);
	return rfx_pol->spike_confirm >= RFX_GAMING_SPIKE_CONFIRM;
}

static void rfx_update_heavy(struct rfx_policy *rfx_pol, bool confirmed, u64 time)
{
	if (rfx_gaming()) {
		rfx_pol->in_deep_idle = false;
		/* Only a CONFIRMED sustained load (or active frame pacing) latches
		 * heavy mode + cluster floors. Unconfirmed lobby bursts do not, so
		 * they never wake every core to f_max. */
		if (confirmed) {
			rfx_pol->in_heavy_mode = true;
			rfx_pol->gaming_lock_end_ns = time + RFX_GAMING_LOCK_NS;
		} else if (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns) {
			rfx_pol->in_heavy_mode = true;
		} else if (!rfx_pol->gaming_lock_end_ns ||
			   (time - rfx_pol->gaming_lock_end_ns) > RFX_GAMING_HYSTERESIS_NS) {
			rfx_pol->in_heavy_mode = false;
		}
		return;
	}

	/* Daily: heavy only while the gaming lock is still draining. */
	if (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns)
		return;
	rfx_pol->gaming_lock_end_ns = 0;
	rfx_pol->in_heavy_mode = false;
}

/* ===================== Headroom ===================== */

static unsigned long rfx_apply_headroom(unsigned long util, unsigned long max_cap, bool is_heavy)
{
	unsigned int pct;

	if (!max_cap)
		return util;

	util = min(util, max_cap);
	pct = (unsigned int)(util * 100 / max_cap);
	if (pct >= 98)
		return max_cap;

	if (rfx_gaming())
		return min(util + util * (is_heavy ? RFX_GAMING_HEADROOM_PCT : 20) / 100, max_cap);

	/* Daily: thrifty headroom, scaling with load. */
	if (rfx_is_little(max_cap)) {
		if (pct >= 70) return min(util + (util >> 4), max_cap);
		if (pct >= 45) return min(util + (util >> 5), max_cap);
		return util;
	}
	if (pct >= 75) return min(util + (util >> 4), max_cap);
	if (pct >= 50) return min(util + (util >> 5), max_cap);
	return min(util + (util >> 6), max_cap);
}

/* ===================== Frequency selection ===================== */

static unsigned int rfx_calc_freq(struct rfx_policy *rfx_pol, unsigned long util,
				  unsigned long max_cap, bool is_heavy, u64 time)
{
	struct cpufreq_policy *policy = rfx_pol->policy;
	bool little = rfx_is_little(max_cap);
	bool prime  = rfx_is_prime(max_cap);
	unsigned int freq, tcap;

	if (!policy || !max_cap)
		return 0;

	util = rfx_apply_headroom(util, max_cap, is_heavy);

	/* Base: classic capacity->frequency map. */
	freq = (unsigned int)((u64)policy->cpuinfo.max_freq * util / max_cap);
	freq = clamp_t(unsigned int, freq, policy->cpuinfo.min_freq, policy->cpuinfo.max_freq);

	if (rfx_gaming()) {
		/* Floors (keep threads off the wall) then caps (leave headroom). */
		if (prime) {
			if (is_heavy)
				freq = max(freq, rfx_floor_of_max(policy, RFX_GAMING_FLOOR_PRIME_PCT));
			freq = min(freq, rfx_pct_of_max(policy, RFX_GAMING_CAP_PRIME_PCT));
		} else if (!little) {
			if (is_heavy)
				freq = max(freq, rfx_floor_of_max(policy, RFX_GAMING_FLOOR_BIG_PCT));
			freq = min(freq, rfx_pct_of_max(policy, RFX_GAMING_CAP_BIG_PCT));
		} else {
			/* Hold a mid floor while heavy so the LITTLE cluster stops
			 * free-falling to ~10% and then snapping back up - that
			 * 10%<->85% oscillation is a major jank source. Floor < cap
			 * still leaves room to idle down between bursts. */
			if (is_heavy)
				freq = max(freq, rfx_floor_of_max(policy, RFX_GAMING_FLOOR_LITTLE_PCT));
			freq = min(freq, rfx_pct_of_max(policy, RFX_GAMING_LITTLE_CAP_PCT));
		}

		/* Frame-overrun floor boost (overrides comfort caps to clear jank). */
		if (rfx_pol->frame_boost_end_ns && time < rfx_pol->frame_boost_end_ns) {
			if (prime)
				freq = max(freq, rfx_floor_of_max(policy, RFX_FRAME_FLOOR_PRIME_PCT));
			else if (!little)
				freq = max(freq, rfx_floor_of_max(policy, RFX_FRAME_FLOOR_BIG_PCT));
		}

		/* Sustain: damp a single-frame util dip from yanking the clock. */
		if (!little) {
			unsigned int hi = rfx_pct_of_max(policy, 80);
			unsigned int lo = rfx_pct_of_max(policy, 75);

			if (freq >= hi)
				rfx_pol->sustain_hold_end_ns = time + RFX_SUSTAIN_HOLD_NS;
			if (time < rfx_pol->sustain_hold_end_ns)
				freq = max(freq, lo);
		}
	} else {
		/* Daily: park to f_min on confirmed idle; keep a small interactive
		 * floor on the big cluster so UI scroll doesn't start cold. */
		if (rfx_pol->in_deep_idle && !is_heavy)
			freq = policy->cpuinfo.min_freq;
		else if (!little && !is_heavy &&
			 rfx_pol->interactive_end_ns && time < rfx_pol->interactive_end_ns)
			freq = max(freq, rfx_floor_of_max(policy, RFX_INTERACTIVE_FLOOR_BIG_PCT));
	}

	/* Thermal cap (predictive + measured). Applies in both modes. */
	tcap = rfx_thermal_cap(rfx_pol, time);
	if (tcap < 100)
		freq = min(freq, rfx_pct_of_max(policy, tcap));

	/* Input boost floor (touch latency). */
	if (rfx_pol->input_boost_end_ns && time < rfx_pol->input_boost_end_ns)
		freq = max(freq, rfx_pol->input_boost_freq);

	freq = min(freq, policy->max);

	if (freq == rfx_pol->cached_raw_freq && !rfx_pol->need_freq_update)
		return rfx_pol->next_freq;

	rfx_pol->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}

/* ===================== Rate limiting ===================== */

static bool rfx_should_update(struct rfx_policy *rfx_pol, u64 time)
{
	s64 delta, delay;
	bool going_up;

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

	going_up = rfx_pol->next_freq < rfx_pol->policy->cur;

	if (rfx_pol->in_deep_idle)
		delay = 3 * NSEC_PER_USEC;
	else if (rfx_gaming() || rfx_pol->in_heavy_mode)
		delay = going_up ? 0 : (s64)RFX_GAMING_DOWN_DELAY_US * NSEC_PER_USEC;
	else
		delay = rfx_pol->freq_update_delay_ns;

	delta = going_up ? (time - rfx_pol->last_upfreq_time)
			 : (time - rfx_pol->last_downfreq_time);
	return delta >= delay;
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
	} else if (rfx_pol->next_freq == next_freq && rfx_pol->last_upfreq_time == time) {
		return false;
	}

	if (next_freq < rfx_pol->next_freq) {
		if (!force_down) {
			s64 dd = time - rfx_pol->last_downfreq_time;
			s64 eff = (rfx_gaming() || rfx_pol->in_heavy_mode)
				? (s64)RFX_GAMING_DOWN_DELAY_US * NSEC_PER_USEC
				: rfx_pol->down_rate_delay_ns;

			if (eff > 0 && dd < eff)
				return false;
		}
		rfx_pol->last_downfreq_time = time;
	} else {
		s64 ud = time - rfx_pol->last_upfreq_time;

		if (rfx_pol->up_rate_delay_ns > 0 && ud < rfx_pol->up_rate_delay_ns)
			return false;
		rfx_pol->last_upfreq_time = time;
	}

	rfx_pol->next_freq = next_freq;
	return true;
}

/* ===================== NOHZ idle hint ===================== */

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

/* ===================== Shared decision core ===================== */

static void rfx_clear_expired(struct rfx_policy *rfx_pol, u64 time)
{
	if (rfx_pol->interactive_end_ns && time >= rfx_pol->interactive_end_ns)
		rfx_pol->interactive_end_ns = 0;
	if (rfx_pol->input_boost_end_ns && time >= rfx_pol->input_boost_end_ns)
		rfx_pol->input_boost_end_ns = 0;
	if (rfx_pol->frame_boost_end_ns && time >= rfx_pol->frame_boost_end_ns)
		rfx_pol->frame_boost_end_ns = 0;
	if (rfx_pol->gaming_lock_end_ns && time >= rfx_pol->gaming_lock_end_ns &&
	    !rfx_gaming())
		rfx_pol->gaming_lock_end_ns = 0;
}

/* Returns selected freq; sets *force_down for an immediate (un-damped) drop. */
static unsigned int rfx_decide(struct rfx_cpu *lead, u64 time, unsigned long pooled_util,
			       unsigned int util_pct, bool *force_down)
{
	struct rfx_policy *rfx_pol = lead->rfx_policy;
	unsigned long max_cap = arch_scale_cpu_capacity(lead->cpu);
	unsigned int ema_pct, eff_pct, next_f;
	unsigned long effective;
	bool is_heavy, confirmed = true;

	ema_pct = rfx_ema_filter(rfx_pol, util_pct);
	eff_pct = max(ema_pct, util_pct);		/* never below the raw demand */

	/* Gaming: hold an unconfirmed high-load burst (lobby asset load) to a
	 * comfort cap so it cannot ramp the cluster to f_max. EMA itself is
	 * untouched - this only bounds the value handed to freq selection. */
	if (rfx_gaming()) {
		confirmed = rfx_spike_confirmed(rfx_pol, util_pct, time);
		if (rfx_pol->burst_capped)
			eff_pct = min(eff_pct, (unsigned int)RFX_GAMING_BURST_CAP_PCT);
	}

	effective = max_cap * eff_pct / 100;
	if (effective < pooled_util && !rfx_pol->burst_capped)
		effective = pooled_util;		/* never below PELT+iowait */

	/* Idle detection (daily only): confirm with the NOHZ idle-call counter
	 * so we only deep-park when the CPU is genuinely going idle, not merely
	 * momentarily low between frames. */
	if (!rfx_gaming()) {
		bool nohz_idle = rfx_nohz_idle(lead);

		if (nohz_idle && rfx_pol->last_real_update_ns &&
		    (time - rfx_pol->last_real_update_ns) > RFX_IDLE_STALE_NS &&
		    util_pct < RFX_IDLE_ENTER_PCT)
			rfx_pol->in_deep_idle = true;
		else if (util_pct >= RFX_IDLE_ENTER_PCT) {
			rfx_pol->in_deep_idle = false;
			rfx_pol->last_real_update_ns = time;
		}
	} else if (util_pct >= RFX_IDLE_ENTER_PCT) {
		rfx_pol->last_real_update_ns = time;
	}

	rfx_update_heavy(rfx_pol, confirmed, time);
	rfx_frame_observe(rfx_pol, time);

	if (!rfx_gaming() && util_pct >= 1) {
		rfx_pol->interactive_end_ns = time + RFX_INTERACTIVE_DURATION_NS;
		rfx_pol->in_deep_idle = false;
	}

	is_heavy = rfx_pol->in_heavy_mode ||
		   (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns);

	next_f = rfx_calc_freq(rfx_pol, effective, max_cap, is_heavy, time);
	rfx_thermal_observe(rfx_pol, next_f, time);

	/* Sustain hold on a sharp drop while heavy (anti-jank). */
	if (is_heavy && lead->prev_util_pct > RFX_BURST_DROP_THRESHOLD &&
	    lead->prev_util_pct > util_pct &&
	    (lead->prev_util_pct - util_pct) >= RFX_BURST_DROP_THRESHOLD &&
	    (!rfx_pol->sustain_hold_end_ns || time > rfx_pol->sustain_hold_end_ns))
		rfx_pol->sustain_hold_end_ns = time + RFX_SUSTAIN_HOLD_NS;
	lead->prev_util_pct = util_pct;

	*force_down = rfx_pol->in_deep_idle && !is_heavy;
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

/* ===================== Util hooks ===================== */

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

/* ===================== Slow-path worker ===================== */

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

/* ===================== sysfs ===================== */

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

static ssize_t input_boost_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->input_boost_ms);
}
static ssize_t input_boost_ms_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->input_boost_ms = val;
	return count;
}
static struct governor_attr input_boost_ms = __ATTR_RW(input_boost_ms);

static ssize_t input_boost_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->input_boost_pct);
}
static ssize_t input_boost_pct_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->input_boost_pct = val;
	return count;
}
static struct governor_attr input_boost_pct = __ATTR_RW(input_boost_pct);

/* Global (cross-policy) knobs, backed by atomics. */
static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&rfx_gaming_mode_global));
}
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct rfx_policy *pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;

	atomic_set(&rfx_gaming_mode_global, val);
	WRITE_ONCE(sched_gaming_active, val);	/* keep the scheduler in sync */
	if (!val) {
		list_for_each_entry(pol, &attr_set->policy_list, tunables_hook) {
			pol->gaming_lock_end_ns = 0;
			pol->input_boost_end_ns = 0;
			pol->frame_boost_end_ns = 0;
			pol->thermal_throttle_active = false;
			pol->thermal_high_start_ns = 0;
			pol->sustain_hold_end_ns = 0;
			pol->in_heavy_mode = false;
			pol->spike_confirm = 0;
			pol->burst_capped = false;
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

	if (kstrtouint(buf, 10, &val) || val < 40 || val > 100)
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
	&input_boost_ms.attr,
	&input_boost_pct.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_big);

static struct attribute *rfx_prime_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&input_boost_ms.attr,
	&input_boost_pct.attr,
	&gaming_mode.attr,
	&thermal_cap_pct.attr,
	&frame_time_us.attr,
	&frame_budget_us.attr,
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

/* ===================== Lifecycle ===================== */

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
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAGS_UGOV,
		.sched_runtime	= 3 * NSEC_PER_MSEC,
		.sched_deadline	= 10 * NSEC_PER_MSEC,
		.sched_period	= 10 * NSEC_PER_MSEC,
	};
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

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("vorpal: failed to set SCHED_DEADLINE\n");
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

	t->input_boost_ms = 150;
	t->input_boost_pct = RFX_INPUT_BOOST_FREQ_PCT;

	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	if (rfx_is_little(max_cap)) {
		t->cluster_type = RFX_CLUSTER_LITTLE;
		t->rate_limit_us = RFX_LITTLE_UP_US;
		t->up_rate_limit_us = RFX_LITTLE_UP_US;
		t->down_rate_limit_us = RFX_LITTLE_DOWN_US;
		ktype = &rfx_little_ktype;
	} else if (rfx_is_prime(max_cap)) {
		t->cluster_type = RFX_CLUSTER_PRIME;
		t->rate_limit_us = 1;
		t->up_rate_limit_us = RFX_PRIME_UP_US;
		t->down_rate_limit_us = RFX_PRIME_DOWN_US;
		ktype = &rfx_prime_ktype;
	} else {
		t->cluster_type = RFX_CLUSTER_BIG;
		t->rate_limit_us = RFX_BIG_UP_US;
		t->up_rate_limit_us = RFX_BIG_UP_US;
		t->down_rate_limit_us = RFX_BIG_DOWN_US;
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
	rfx_pol->input_boost_end_ns = 0;
	rfx_pol->input_boost_freq = 0;
	rfx_pol->frame_boost_end_ns = 0;
	rfx_pol->sustain_hold_end_ns = 0;
	rfx_pol->interactive_end_ns = 0;
	rfx_pol->thermal_high_start_ns = 0;
	rfx_pol->thermal_throttle_end_ns = 0;
	rfx_pol->thermal_throttle_active = false;
	rfx_pol->in_deep_idle = false;
	rfx_pol->last_real_update_ns = now;
	rfx_pol->in_heavy_mode = false;
	rfx_pol->ema_util_pct = 0;
	rfx_pol->spike_confirm = 0;
	rfx_pol->burst_capped = false;

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

static int __init vorpal_init(void)
{
	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION, CPUFREQ_VORPAL_AUTHOR);
	return cpufreq_register_governor(&vorpal_gov);
}

static void __exit vorpal_exit(void)
{
	cpufreq_unregister_governor(&vorpal_gov);
}

module_init(vorpal_init);
module_exit(vorpal_exit);

MODULE_AUTHOR(CPUFREQ_VORPAL_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v4.0 - Gaming & Efficiency");
