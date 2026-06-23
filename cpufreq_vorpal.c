// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.0 - Perfect Gaming & Thermal Edition
 * Based on schedutil — optimized for 120fps gaming & daily use
 *
 * Features:
 *   • Dual-Profile Operating Modes      	— Gaming (locked high-band) / Daily (power-efficient)
 *   • Tri-Cluster Topology Awareness    	— Independent tuning for Little / Big / Prime
 *   • Directional EMA Util Smoothing    	— Fast-rise / slow-decay anti-yoyo filter
 *   • Dynamic Capacity Headroom         	— Load-proportional OPP headroom allocation
 *   • Proactive Thermal Step Controller 	— Smooth 2%-down / 1%-up cap ramping
 *   • Thermal Zone Integration          	— Hardware sensor + userspace fallback
 *   • Frame Pacing & Miss Recovery      	— Bounded floor boost on 120fps overrun
 *   • Global Frame Boost                	— All-cluster sync on dropped frames
 *   • Touch Input Responsiveness Boost  	— 220ms touch-window floor lift
 *   • UI Ramp-Assist / Render Burst     	— Sharp util-rise detection for animations
 *   • Adaptive Floor (Idle/Busy)        	— Prime & Little dynamic floor switching
 *   • Directional Rate Limiting         	— Per-cluster up/down rate gates
 *   • IOWait Performance Boost          	— Schedutil-legacy IOWait handling
 *   • Deadline Bandwidth Awareness      	— DL task frequency bypass
 *   • Jank Telemetry & Statistics       	— Frame/jank ratio reporting
 *   • Deferred IRQ-Work Frequency Commit 	— Async non-fast-switch path
 *   • Global Policy State Reset         	— Clean gaming-off transition
 *   • GKI 5.10 Util Interface           	— rfx_get_util_gki510 / rfx_dl_bw_exceeded_gki510
 *   • Scheduler Coupling              		— BORE/CFS gaming biases via sched_gaming_active
 *
 * Author: Templar Dev (Steambot12)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/topology.h>
#include <linux/rcupdate.h>
#include <linux/sched/rt.h>
#include <linux/sched/cpufreq.h>
#include <uapi/linux/sched/types.h>
#include <linux/tick.h>
#include <linux/timekeeping.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/irq_work.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/list.h>
#ifdef CONFIG_THERMAL
#include <linux/thermal.h>
#endif

#define CPUFREQ_VORPAL_NAME     "vorpal"
#define CPUFREQ_VORPAL_VERSION  "2.0"
#define CPUFREQ_VORPAL_AUTHOR   "Templar Dev"

/*
 * Scheduler coupling symbol. Defined and EXPORT_SYMBOL_GPL'd in
 * kernel/sched/fair.c (file-scope int, NOT a task_struct field -> KMI safe).
 * Written here by gaming_mode_store; read by fair.c gaming biases.
 */
extern int sched_gaming_active;

/* Core-sched util getter / deadline-bandwidth check (owned by core sched). */
extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);

/*
 * Built-in frame pacing source. drm_vblank.c publishes the timestamp of the
 * last DELIVERED present/flip at its send_vblank_event() chokepoint (one event
 * = one presented frame), so frame timing is automatic whenever gaming_mode=1
 * and the panel flips - no userspace feeder, no root, no dumpsys. DRM only
 * publishes; the governor only consumes (no reverse dependency). Fallback to a
 * local zero atomic if DRM is not built in.
 */
#if IS_ENABLED(CONFIG_DRM)
extern atomic64_t drm_last_present_ns;
#else
static atomic64_t drm_last_present_ns = ATOMIC64_INIT(0);
#endif

/* ===================================================================== */
/* Tunable defaults (KMI-safe: plain #defines, no struct-layout changes). */
/* ===================================================================== */

/* Cluster identification by arch capacity. */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* Per-cluster rate limits (microseconds). up=0 means "scale up instantly". */
#define RFX_LITTLE_RATE_US		1000
#define RFX_LITTLE_UP_US		0
#define RFX_LITTLE_DOWN_US		4000

#define RFX_BIG_RATE_US			500
#define RFX_BIG_UP_US			0
#define RFX_BIG_DOWN_US			8000

#define RFX_PRIME_RATE_US		500
#define RFX_PRIME_UP_US			0
#define RFX_PRIME_DOWN_US		8000

/*
 * Gaming down-rate-limit. While gaming, frequency may only step DOWN this
 * slowly (microseconds). Combined with the floors below this is what kills
 * the yoyo / sawtooth: the clock parks in the high band and decays gently.
 *
 * 150ms = the value from the user's PROVEN-STABLE "PERFECT BUILD" commit
 * (95c3e68: jank 1%, avg 117.7fps, 5.3W, <40C). That build's whole theme was a
 * STEADY gaming clock - hold high, decay slowly - and a long down-rate is the
 * single biggest contributor. Safe thermally: the down-rate only acts while
 * freq is ABOVE demand and descending; the sustained heat we measured comes
 * from genuine util-pegging (demand), which a slow down-rate does NOT add to.
 * Up stays instant (up-rate 0 + EMA instant-attack), so a real spike is served
 * immediately; only the DECAY is slowed -> fast-attack / slow-decay = steady.
 */
#define RFX_GAMING_DOWN_US		150000

/* ---- Gaming frequency band, percent of policy fmax ---- */
/*
 * All gaming caps are 100% (= no artificial ceiling). With the double-headroom
 * removed (see rfx_apply_headroom), the requested frequency now equals clean
 * schedutil demand: light/medium frames request a low OPP and only a genuinely
 * heavy frame reaches fmax, finishes fast and returns to idle (race-to-idle).
 * A sub-100 cap would instead slow a saturated core, which RAISES load% at
 * fixed work and starves frames - the opposite of "sustain". The floors below
 * keep the cores warm/ready; the thermal step clamp is the real ceiling.
 *
 * Prime is a single-core cluster: the render thread hops in/out of it, so the
 * floor is adaptive - locked high only while Prime is genuinely busy, dropped
 * to a lower idle floor otherwise (up-rate is 0 so it snaps back instantly).
 */
#define RFX_G_PRIME_FLOOR_PCT		90	/* busy: locked high */
#define RFX_G_PRIME_IDLE_FLOOR_PCT	78	/* idle: stay warm (~2.3GHz) - skin runs cool (ap_ntc ~40C), so a higher idle floor keeps Prime ready for an incoming render thread and kills the cold-start dip, with no thermal risk */
#define RFX_G_PRIME_BUSY_ENTER_PCT	20	/* latch busy-hold on light activity too: a bursty render thread (Delta Force) then lands on a WARM Prime (busy floor) instead of the 2400 idle floor - fixes the cold-start dip + the inter-burst sag. Safe: Prime is a single core, so the extra warmth is bounded to one CPU (unlike the 3-core Big, which overheated) */
#define RFX_G_PRIME_CAP_PCT		100
#define RFX_G_PRIME_FRAME_PCT		92	/* gentle lift above the busy floor; below fmax (race-to-idle) */
/*
 * Busy-hold hysteresis: once Prime is busy, keep it locked at the high floor
 * for this long after the last busy sample. A bursty game thread (Delta Force)
 * pulses Prime load above/below BUSY_ENTER; without a hold the floor toggled
 * down into every inter-burst gap, oscillating Prime 2.0<->3.0GHz (frame-time
 * + power spikes). The hold bridges those gaps so Prime stays steady; only a
 * genuinely parked Prime (idle past the hold) relaxes to the idle floor.
 */
#define RFX_G_PRIME_HOLD_NS		(200 * NSEC_PER_MSEC)
/*
 * Cold-start warmup: for this long after gaming_mode=1, Prime is treated as busy
 * so it starts at the high floor. At match load the render thread is not yet
 * established (util low) -> without this Prime sits at the 2400 idle floor and
 * the first frames land on a cold core (the recurring start-of-match dip). One
 * single core held high for a few seconds = negligible heat.
 */
#define RFX_G_PRIME_WARMUP_NS		(3000 * NSEC_PER_MSEC)
/*
 * Big: the render thread lives mostly HERE, but a HIGH flat floor backfires -
 * 3 Big cores pinned high run hot, and on this device that heat trips the VENDOR
 * step_wise thermal governor (cap_pct stays 100, so it is not us), which slams
 * policy->max -> freq forced below demand -> load spikes ~98% -> FPS drop. So a
 * floor of 92% measured WORSE than 85%. The right model after the double-headroom
 * fix is race-to-idle: a moderate floor (anti-sag only) + instant EMA attack +
 * zero up-rate, so Big jumps to fmax on a real demand spike and idles between =
 * coolest, stays under the vendor trip, sustains FPS. 85% is the cool-stable edge.
 */
#define RFX_G_BIG_FLOOR_PCT		85	/* ~2.0GHz anti-sag; race-to-idle does the rest */
#define RFX_G_BIG_CAP_PCT		100
#define RFX_G_BIG_FRAME_PCT		90	/* >floor: lift on a frame miss, but below fmax */
/*
 * Little: support work, no cap. A latency-sensitive game worker stuck on a
 * 600MHz LITTLE finishes ~3x slower than at mid OPP and can land a frame late,
 * so during gaming LITTLE holds a mid-range floor (~1.2GHz) instead of dropping
 * to fmin. The ENTER gate is low (12%) so only a genuinely idle LITTLE sleeps
 * down - this kills the frequent 600MHz crashes seen under load while the huge
 * thermal headroom (skin ~37C, ~5.4W) easily absorbs the extra LITTLE power.
 */
#define RFX_G_LITTLE_CAP_PCT		100
#define RFX_G_LITTLE_FLOOR_PCT		58	/* ~1.05GHz mid OPP - above the 600MHz cliff, cooler than 65 */
#define RFX_G_LITTLE_FLOOR_ENTER_PCT	12	/* apply floor unless near-idle */
#define RFX_G_LITTLE_FRAME_PCT		65	/* gentle: support-cluster recovery lift */
/*
 * LITTLE busy-hold: same hysteresis idea as Prime, tuned separately. A momentary
 * util dip below FLOOR_ENTER used to drop the floor and let LITTLE crash to fmin
 * (600MHz) mid-gaming - a latency-sensitive worker landing there then stalled.
 * Holding the floor for this long after the last busy sample bridges the
 * micro-idle gaps so LITTLE stays on its ~1.2GHz floor through gameplay.
 */
#define RFX_G_LITTLE_HOLD_NS		(80 * NSEC_PER_MSEC)

/* ---- Daily frequency shaping, percent of policy fmax ---- */
/*
 * "UI active" = a touch is recent OR a render burst was just detected (see
 * the ramp-assist below). When active, LITTLE's cap is relaxed and LITTLE/Big
 * get a responsiveness floor so animations / captions / scrolls do not run at
 * a starved OPP. When idle, LITTLE is capped low for battery.
 */
/*
 * Daily LITTLE idle cap. The old 65% starved UI work that the "UI-active"
 * heuristic missed - a non-touch burst that rises GRADUALLY (a smooth animation,
 * a video subtitle redraw, sustained moderate-util drawing) never trips the
 * touch window or the >=12% ramp, so LITTLE stayed pinned at 65% -> stutter,
 * while touch/sharp bursts got boosted -> smooth. That is the "sometimes smooth,
 * sometimes not" daily UI. Raising the cap to 78% removes that starvation
 * ceiling for undetected work. Battery cost is ~nil: the cap only bites when
 * LITTLE util is genuinely high (real work); at true idle, util-following keeps
 * the freq low regardless of the cap. UI-active still relaxes further to 90%.
 */
#define RFX_D_LITTLE_CAP_PCT		78	/* idle cap (was 65): stop starving undetected UI bursts */
#define RFX_D_LITTLE_BOOST_CAP_PCT	90	/* relax cap while UI active */
#define RFX_D_LITTLE_UI_FLOOR_PCT	58	/* LITTLE floor while UI active */
#define RFX_D_BIG_UI_FLOOR_PCT		55	/* Big floor while UI active */

/*
 * Daily UI ramp-assist. PELT/WALT util lags the real frame work, and the most
 * stutter-prone UI moments (a video caption appearing, an app open/close
 * animation, a fling-scroll) are often NOT touch events, so a touch-only boost
 * misses them. Instead we watch the smoothed util for a sharp RISE: a jump of
 * >= RFX_D_RAMP_DELTA_PCT points arms a short floor that holds for
 * RFX_D_UI_BOOST_NS, re-arming as long as demand keeps climbing. This lifts the
 * first frames of a burst immediately, independent of input - the core fix for
 * the gaming_mode=0 display/UI stutter.
 */
#define RFX_D_RAMP_DELTA_PCT		12
#define RFX_D_UI_BOOST_NS		(150 * NSEC_PER_MSEC)

/*
 * Touch window, lengthened 90 -> 220 ms so a tap-initiated app open/close
 * animation (~300 ms) stays boosted through the whole transition instead of
 * sagging halfway when the old short window expired.
 */
#define RFX_INPUT_WINDOW_NS		(220 * NSEC_PER_MSEC)

/* ---- Util EMA (directional smoothing). new>old: up_shift, else down_shift ---- */
#define RFX_EMA_UP_SHIFT_DAILY		0	/* instant attack: a render-thread burst (texture upload / compositing) gets clock immediately -> shorter RT spike -> less daily frame-time jitter. Decay stays slow (down-shift 3), so ~no battery cost */
#define RFX_EMA_DN_SHIFT_DAILY		3	/* decay gently: no inter-frame sag */
#define RFX_EMA_UP_SHIFT_GAMING		0	/* instant attack: no lag on a render spike */
#define RFX_EMA_DN_SHIFT_GAMING		3	/* decay slowly -> anti-jitter */

/*
 * ---- Headroom ----
 * Gaming adds NO governor-side headroom: rfx_get_util_gki510() already applies
 * the standard 25% DVFS headroom. Daily uses a small tiered top-up (hardcoded
 * shifts in rfx_apply_headroom) that adds little at low util (battery) and more
 * as util climbs (responsiveness).
 */

/* ---- Thermal step controller ---- */
#define RFX_THERMAL_STEP_NS		(6 * NSEC_PER_MSEC)
#define RFX_THERMAL_STEP_DOWN_PCT	2
#define RFX_THERMAL_STEP_UP_PCT		1
#define RFX_THERMAL_MIN_CAP_PCT		70
#define RFX_THERMAL_POLL_GAMING_MS	50
#define RFX_THERMAL_POLL_IDLE_MS	200
/*
 * Temperature breakpoints (milli-Celsius) -> target cap percent, tuned for a
 * SKIN / board sensor (bind one via the thermal_zone sysfs). The goal is a
 * gentle pre-emptive shave that begins BEFORE the vendor step_wise governor
 * reaches its hard policy->max trip, so heat is bled off smoothly instead of in
 * FPS-crashing slams: stay 100% through normal gameplay (< GREEN), a shallow
 * -2%/C up to YELLOW, a firmer -3%/C up to RED, then hold the floor and let the
 * vendor framework handle any genuine emergency beyond. The floor is high (75%)
 * and RFX_THERMAL_MIN_CAP_PCT guards it, so a heavy frame always has clock to
 * land on - this is a smoother, not a deeper, throttle than the vendor's.
 *
 * NOTE: SKIN-scale values. Do NOT bind a CPU-junction sensor here - it reads
 * 50-70C in normal play and would throttle mid-match (the v6.11 regression).
 */
/*
 * Breakpoints sit ABOVE the device's normal gaming skin temp (benchmarks show
 * ap_ntc peaks ~45-47C in a long PUBG session). Keeping GREEN at 48C means the
 * cap stays 100% through all normal play - so the governor does NOT shave a
 * healthy frame (an over-early shave was measured to raise jank ~1.9% -> ~3%).
 * The gentle -2/-3%/C ramp only engages past 48C as a pre-emptive safety net,
 * bleeding heat smoothly before the vendor step_wise governor's hard slam.
 */
#define RFX_TEMP_GREEN_MC		48000	/* hold 100% through normal play */
#define RFX_TEMP_YELLOW_MC		52000	/* -2%/C below, -3%/C above */
#define RFX_TEMP_RED_MC			56000	/* floor reached; vendor beyond */

/* ---- Frame pacing ---- */
#define RFX_FRAME_BUDGET_US_120		8333	/* 1e6/120 */
#define RFX_FRAME_BUDGET_US_90		11111	/* 1e6/90  */
/*
 * Default gaming frame budget = 90fps. This is the PROVEN-STABLE value (28-min
 * PUBG: jank 1.7%, min 74.9, avg 117.7, 46.5C, 5.86W). Setting it to 120fps was
 * measured to be MUCH WORSE: at 8333us the proactive-sustain arms the instant
 * frame time slides past ~8.83ms (~113fps), i.e. for the whole 113-120 band, so
 * on a device already near its thermal envelope it chased every shallow sag ->
 * extra clock -> faster junction climb -> earlier vendor throttle -> MORE drops.
 * The recurring wall: pushing the governor to defend 120 harder costs more heat
 * than it gives. At 11111us/90fps the sustain stays dormant through normal 120fps
 * play (frames look on-time vs the larger budget) so the clock races-to-idle and
 * stays cool; it only wakes for a genuinely deep sag (< ~85fps). The remaining
 * shallow drops on this build are single heavy frames near the cpufreq floor -
 * the real lever for fully removing them is the in-game FPS cap, not the budget.
 * Userspace can still pin any value via the frame_budget_us sysfs (a middle 100fps
 * = echo 10000 catches the deeper sags without the 120-budget heat, if wanted).
 */
#define RFX_FRAME_BUDGET_US_GAMING	RFX_FRAME_BUDGET_US_90
/*
 * Recovery-boost window. SHORT on purpose (~4 frames at 120fps): a title that
 * chronically misses its target would, with a long window, keep the boost
 * permanently armed -> clusters pinned high -> heat -> vendor throttle -> the
 * very FPS drops the boost tried to prevent (the v6.10 regression). 33ms means
 * the boost is a brief nudge, not a latch.
 */
#define RFX_FRAME_BOOST_NS		(33 * NSEC_PER_MSEC)
/*
 * Anti-runaway GATE - the key safety. Only arm the recovery boost if the render
 * (Big) cluster still has frequency headroom (running below this percent of
 * fmax). A frame miss while Big is ALREADY near fmax is GPU / IO / thermal /
 * placement bound, NOT frequency bound, so boosting would add heat for no FPS
 * gain and trip the vendor thermal governor. Gating here means the feeder can
 * never pin an already-busy cluster: it only fills in clock where recovery is
 * physically possible (cold-start, scene transitions), so it cannot regress
 * below the cool race-to-idle baseline.
 */
#define RFX_FRAME_BOOST_GATE_PCT	90
/* Ignore present intervals longer than this (app switch / resume / paused). */
#define RFX_FRAME_PRESENT_GAP_NS	(200 * NSEC_PER_MSEC)
#define RFX_JANK_WINDOW_NS		(2000 * NSEC_PER_MSEC)

/*
 * Proactive frame-margin SUSTAIN. The reactive boost above only fires once a
 * frame has already overrun 1.5x budget (a real jank, FPS already ~80). To hold
 * a steady 120fps with min ~117, we must also catch frames that are MEROSOT -
 * sliding below the target but not yet a jank. We watch a SMOOTHED frame time
 * (rfx_ft_ema_us, EMA below) and arm the same bounded, gated boost as soon as it
 * creeps past this percent of budget. 106% of an 8333us budget = ~113fps, so the
 * sustain floor engages while FPS is dipping toward - but still above - 117, and
 * pulls it back before the dip is visible. CRUCIAL non-greedy property: when the
 * game is truly on-cadence the smoothed frame time sits at ~100% of budget
 * (one present per vsync), which is BELOW this threshold, so the feature is
 * OFF and the clusters stay in race-to-idle - average CPU load is unchanged.
 * It only spends power on the edge, never in easy scenes. Raise toward 110-115
 * to make it lazier (less power, fewer interventions); lower toward 103 to chase
 * 120 harder (more power). The same headroom gate + 33ms window + below-fmax
 * boost floors bound it, so it cannot run away the way the v7.x perf-loops did.
 */
#define RFX_FRAME_SUSTAIN_PCT		106	/* arm proactively when EMA frame time > this % of budget (~113fps) */
#define RFX_FT_EMA_SHIFT		2	/* frame-time EMA: alpha = 1/4 (anti single-frame noise) */

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

enum rfx_cluster_type {
	RFX_CLUSTER_LITTLE = 0,
	RFX_CLUSTER_BIG,
	RFX_CLUSTER_PRIME,
};

/* ===================================================================== */
/* Global state                                                          */
/* ===================================================================== */

/* Master gaming switch, written by gaming_mode sysfs (Prime cluster only). */
static atomic_t rfx_gaming = ATOMIC_INIT(0);

static inline bool rfx_gaming_enabled(void)
{
	return atomic_read(&rfx_gaming) != 0;
}

/* Last input event timestamp (daily touch boost). */
static atomic64_t rfx_input_ts_ns = ATOMIC64_INIT(0);

/* Thermal: target cap published by the poller, consumed by the fast path. */
static atomic_t rfx_thermal_cap_pct = ATOMIC_INIT(100);
/* Userspace-fed temperature fallback (milli-Celsius); 0 = unavailable. */
static atomic_t rfx_temp_mc = ATOMIC_INIT(0);

/* Frame pacing telemetry (userspace feeder writes frame_time_us). */
static atomic_t rfx_frame_time_us = ATOMIC_INIT(0);
static atomic_t rfx_frame_budget_us = ATOMIC_INIT(RFX_FRAME_BUDGET_US_GAMING);
/* True once userspace pins frame_budget_us via sysfs -> gaming won't auto-set it. */
static bool rfx_frame_budget_user_set;
static atomic_t rfx_frames_seen = ATOMIC_INIT(0);
static atomic_t rfx_janks_seen = ATOMIC_INIT(0);
static atomic_t rfx_jank_pct = ATOMIC_INIT(0);
/* Smoothed frame time (us) for the proactive sustain controller. 0 = unset. */
static atomic_t rfx_ft_ema_us = ATOMIC_INIT(0);

/*
 * Frame-miss boost deadline (ns since boot), GLOBAL so every cluster reacts to
 * a dropped frame - not just Prime. A missed frame means SOME cluster was too
 * slow; since the render thread's placement is not known in-kernel, all of
 * Prime/Big/Little lift their floor together until this deadline.
 */
static atomic64_t rfx_frame_boost_end_ns = ATOMIC64_INIT(0);

/* Deadline until which Prime is force-held high after gaming starts (cold-start). */
static atomic64_t rfx_gaming_warmup_end_ns = ATOMIC64_INIT(0);

/*
 * Last DRM present timestamp already consumed by frame accounting. The fast
 * path runs far faster than the frame rate, and every gaming cluster calls
 * rfx_frame_account, so this cmpxchg'd value claims each presented frame
 * exactly once (no double-counting jank%, no re-arming the boost every update).
 */
static atomic64_t rfx_last_present_consumed = ATOMIC64_INIT(0);

/*
 * Per-perf-cluster requested frequency as a percent of fmax, published each
 * gaming update. The frame-boost gate reads BOTH: it arms only when at least
 * one perf cluster still has headroom (min < gate). A miss while BOTH are near
 * fmax is GPU/IO/thermal/placement bound (boost would be heat-only); a miss
 * while one cluster is cold (e.g. the render thread just migrated onto an
 * idle-floored Prime) is exactly the recoverable case the boost exists for.
 * 0 until that cluster runs (treated as headroom = arm, the safe default).
 */
static atomic_t rfx_sys_perf_pct = ATOMIC_INIT(0);	/* Big (render) cluster */
static atomic_t rfx_prime_perf_pct = ATOMIC_INIT(0);	/* Prime cluster */

/* All live policies, so gaming-off can reset every cluster (not just Prime). */
static LIST_HEAD(rfx_policy_list);
static DEFINE_SPINLOCK(rfx_policy_list_lock);

/* ===================================================================== */
/* Data structures                                                       */
/* ===================================================================== */

struct rfx_tunables {
	struct gov_attr_set attr_set;
	unsigned int rate_limit_us;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
	enum rfx_cluster_type cluster_type;
	unsigned int gaming_mode;
};

struct rfx_policy {
	struct cpufreq_policy *policy;
	struct rfx_tunables *tunables;
	struct list_head tunables_hook;
	struct list_head gov_node;	/* on rfx_policy_list */

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

	bool is_prime;			/* this policy is the Prime cluster */
	bool is_little;

	unsigned int prev_upct;		/* last util%, for daily ramp detect */
	u64 ui_boost_end_ns;		/* daily: UI render-burst floor hold */
	u64 prime_busy_hold_ns;		/* gaming: hold Prime high across burst gaps */
	u64 little_busy_hold_ns;	/* gaming: hold LITTLE floor across micro-idle */

	int thermal_applied_pct;	/* walked toward rfx_thermal_cap_pct */
	u64 thermal_step_ns;
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
	unsigned long filt_util;	/* directional EMA of effective util */
};

static DEFINE_PER_CPU(struct rfx_cpu, rfx_cpu);

static inline struct rfx_tunables *to_rfx_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct rfx_tunables, attr_set);
}

static inline struct gov_attr_set *rfx_to_gov_attr_set(struct kobject *kobj)
{
	return container_of(kobj, struct gov_attr_set, kobj);
}

static inline bool rfx_cap_is_little(unsigned long cap)
{
	return cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool rfx_cap_is_prime(unsigned long cap)
{
	return cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
}

/* fmax * pct / 100 */
static inline unsigned int rfx_pct(unsigned int fmax, unsigned int pct)
{
	return (unsigned int)((u64)fmax * pct / 100);
}

static inline bool rfx_input_active(u64 time)
{
	u64 ts = (u64)atomic64_read(&rfx_input_ts_ns);

	return ts && (time - ts) < RFX_INPUT_WINDOW_NS;
}

/* ===================================================================== */
/* Util smoothing                                                        */
/* ===================================================================== */

/*
 * Directional EMA. Rising demand is tracked quickly (small up_shift) so the
 * UI / a new frame is not starved; falling demand decays slowly (larger
 * down_shift) so the clock does not chase every micro-dip - that decay is the
 * core of the anti-jitter / anti-yoyo behaviour. A forced minimum step of 1
 * prevents the filter from stalling on tiny deltas.
 */
static unsigned long rfx_ema(unsigned long old, unsigned long val, bool gaming)
{
	unsigned long up = gaming ? RFX_EMA_UP_SHIFT_GAMING : RFX_EMA_UP_SHIFT_DAILY;
	unsigned long dn = gaming ? RFX_EMA_DN_SHIFT_GAMING : RFX_EMA_DN_SHIFT_DAILY;
	unsigned long diff;

	if (!old)
		return val;
	if (val > old) {
		diff = val - old;
		return old + max(diff >> up, 1UL);
	}
	if (val < old) {
		diff = old - val;
		return old - max(diff >> dn, 1UL);
	}
	return val;
}

/*
 * Headroom: request slightly more capacity than measured so we land on an OPP
 * with room to spare (avoids running pinned at 100% util, which is both a
 * latency and a load-percent problem). Gaming adds nothing here - the util
 * getter already applied the 25% DVFS headroom, and double-counting it would
 * push moderate frames to fmax/top-OPP (heat -> throttle -> frame drops).
 * Daily uses a tiered curve that adds little at low util (battery) and more as
 * util climbs (responsiveness).
 */
static unsigned long rfx_apply_headroom(unsigned long util, unsigned long max_cap,
					bool gaming, bool little)
{
	unsigned int upct;

	if (!max_cap || util >= max_cap)
		return max_cap;

	/* Gaming: util is already headroomed by the getter - pass through. */
	if (gaming)
		return util;

	upct = (unsigned int)(util * 100 / max_cap);
	if (upct >= 95)
		return max_cap;

	if (little) {
		if (upct >= 70)
			return min(util + (util >> 4), max_cap);
		if (upct >= 45)
			return min(util + (util >> 5), max_cap);
		return util;
	}

	if (upct >= 75)
		return min(util + (util >> 4), max_cap);
	if (upct >= 50)
		return min(util + (util >> 5), max_cap);
	return min(util + (util >> 6), max_cap);
}

/* ===================================================================== */
/* Thermal step controller (final clamp)                                 */
/* ===================================================================== */

/* Map temperature (milli-Celsius) to a target cap percent. */
static int rfx_temp_to_cap(int t_mc)
{
	if (t_mc < RFX_TEMP_GREEN_MC)
		return 100;
	if (t_mc < RFX_TEMP_YELLOW_MC)			/* -2%/C -> 90 @ YELLOW */
		return 100 - (t_mc - RFX_TEMP_GREEN_MC) * 2 / 1000;
	if (t_mc < RFX_TEMP_RED_MC)			/* -3%/C, continuous from 90 */
		return 90 - (t_mc - RFX_TEMP_YELLOW_MC) * 3 / 1000;
	return 75;
}

/*
 * Walk the applied cap toward the published target in small, rate-limited
 * steps and clamp the requested frequency to it. Stepping down faster than up
 * gives a smooth throttle entry and a gentle recovery (no oscillation at the
 * trip point). This is the LAST clamp, so it always wins over the floors -
 * that is what bounds power/temperature under the FPS-first policy.
 */
static unsigned int rfx_thermal_clamp(struct rfx_policy *p, unsigned int freq,
				      unsigned int fmax, u64 time)
{
	int target = atomic_read(&rfx_thermal_cap_pct);
	int applied = p->thermal_applied_pct ? p->thermal_applied_pct : 100;

	if ((s64)(time - p->thermal_step_ns) >= (s64)RFX_THERMAL_STEP_NS) {
		if (applied > target)
			applied -= RFX_THERMAL_STEP_DOWN_PCT;
		else if (applied < target)
			applied += RFX_THERMAL_STEP_UP_PCT;
		applied = clamp(applied, RFX_THERMAL_MIN_CAP_PCT, 100);
		p->thermal_applied_pct = applied;
		p->thermal_step_ns = time;
	}

	if (applied < 100) {
		unsigned int cap = rfx_pct(fmax, applied);

		if (freq > cap)
			freq = cap;
	}
	return freq;
}

/* ===================================================================== */
/* Frame pacing                                                          */
/* ===================================================================== */

/*
 * Derive this update's frame time (microseconds), consuming each frame exactly
 * once across all clusters. A userspace feeder write to frame_time_us takes
 * priority (benchmark / override path); otherwise the built-in DRM present feed
 * is used: the interval between two consecutive delivered present events = one
 * frame's display time, automatically, with no userspace at all.
 */
static unsigned int rfx_frame_time_sample(void)
{
	unsigned int ft = atomic_xchg(&rfx_frame_time_us, 0);
	u64 present, prev, delta;

	if (ft)
		return ft;			/* userspace override wins */

	present = (u64)atomic64_read(&drm_last_present_ns);
	if (!present)
		return 0;			/* no DRM feed (yet) */

	prev = (u64)atomic64_read(&rfx_last_present_consumed);
	if (present == prev)
		return 0;			/* already consumed this frame */

	/* Claim this present exactly once - losers of the race bail out. */
	if ((u64)atomic64_cmpxchg(&rfx_last_present_consumed, prev, present)
	    != prev)
		return 0;

	if (!prev || present <= prev)
		return 0;			/* first sample: no interval yet */

	delta = present - prev;
	if (delta > RFX_FRAME_PRESENT_GAP_NS)
		return 0;			/* app switch / resume gap */

	return (unsigned int)(delta / NSEC_PER_USEC);
}

/*
 * Called each gaming update from any cluster. If a presented frame overran 1.5x
 * the budget, arm a SHORT floor boost so the next frames recover - but only if
 * the render cluster still has headroom (the gate). The boost lifts the floor
 * (see target_freq), never forces fmax, so load stays mid and heat stays bounded.
 */
static void rfx_frame_account(u64 time)
{
	unsigned int ft = rfx_frame_time_sample();
	unsigned int bud = atomic_read(&rfx_frame_budget_us);
	int ema;
	bool jank, sag, want_boost;

	if (!ft || !bud)
		return;

	atomic_inc(&rfx_frames_seen);

	/*
	 * Smooth the frame time so a single noisy present does not drive control;
	 * the proactive sustain decision runs off this EMA, not the raw sample.
	 */
	ema = atomic_read(&rfx_ft_ema_us);
	ema = ema ? ema + (((int)ft - ema) >> RFX_FT_EMA_SHIFT) : (int)ft;
	atomic_set(&rfx_ft_ema_us, ema);

	/* Reactive: a real jank (raw frame already overran 1.5x budget). */
	jank = ft > bud + (bud >> 1);
	if (jank)
		atomic_inc(&rfx_janks_seen);

	/*
	 * Proactive sustain: the SMOOTHED frame time has crept into the sag band
	 * (FPS sliding below ~113 but not yet a jank). Catch it BEFORE it becomes
	 * a visible drop so min-FPS holds near the 120 target. Off entirely when
	 * frames are on-cadence (ema ~ budget < threshold) -> easy scenes stay in
	 * race-to-idle and average CPU load is unchanged (not greedy).
	 */
	sag = (s64)ema * 100 > (s64)bud * RFX_FRAME_SUSTAIN_PCT;

	want_boost = jank || sag;
	if (want_boost) {
		/*
		 * Anti-runaway gate over BOTH perf clusters: arm only if the
		 * LEAST-loaded one still has headroom. If both Big and Prime are
		 * already near fmax, the device is GPU/IO/thermal/placement bound
		 * and a boost would be heat-only (the v6.10 runaway) - so leave
		 * frequency alone (jank, if any, is still counted). If one is cold
		 * (a render thread just landed on an idle-floored Prime while Big
		 * is busy, or a cluster sagging in the margin band) we DO arm, so
		 * it lifts to its bounded frame floor and FPS recovers. Lifting a
		 * cluster that is already busy is a no-op via max(), so no extra
		 * heat lands on a saturated core.
		 */
		unsigned int lo = min(atomic_read(&rfx_sys_perf_pct),
				      atomic_read(&rfx_prime_perf_pct));

		if (lo < RFX_FRAME_BOOST_GATE_PCT)
			atomic64_set(&rfx_frame_boost_end_ns,
				     time + RFX_FRAME_BOOST_NS);
	}
}

static inline bool rfx_frame_boost_active(u64 time)
{
	u64 end = (u64)atomic64_read(&rfx_frame_boost_end_ns);

	return end && time < end;
}

/* ===================================================================== */
/* Frequency decision                                                    */
/* ===================================================================== */

/*
 * Pure-ish frequency selection from a (smoothed) util value. Order:
 *   1. headroom -> base freq from util/capacity
 *   2. profile shaping (gaming band lock OR daily caps/floors)
 *   3. thermal step clamp (final ceiling)
 *   4. resolve to a real OPP (cached to skip redundant table walks)
 */
static unsigned int rfx_target_freq(struct rfx_policy *p, unsigned long util,
				    unsigned long max_cap, u64 time, bool gaming)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int fmax = pol->cpuinfo.max_freq;
	unsigned int fmin = pol->cpuinfo.min_freq;
	bool little = rfx_cap_is_little(max_cap);
	bool prime = rfx_cap_is_prime(max_cap);
	unsigned int freq, upct;

	if (!fmax)
		return pol->cur;

	util = rfx_apply_headroom(util, max_cap, gaming, little);
	upct = max_cap ? (unsigned int)(util * 100 / max_cap) : 0;

	freq = (unsigned int)((u64)fmax * util / max_cap);
	freq = clamp(freq, fmin, fmax);

	if (gaming) {
		bool fboost = rfx_frame_boost_active(time);

		if (prime) {
			/*
			 * Adaptive floor + busy-hold hysteresis. Lock Prime high
			 * while busy and KEEP it locked for RFX_G_PRIME_HOLD_NS
			 * after the last busy sample, so a bursty game thread does
			 * not let the floor toggle down into the inter-burst gaps
			 * (that toggling oscillated Prime 2.0<->3.0GHz). Only a
			 * genuinely parked Prime relaxes to the idle floor.
			 */
			bool busy;
			unsigned int fl, cap;

			if (upct >= RFX_G_PRIME_BUSY_ENTER_PCT)
				p->prime_busy_hold_ns = time + RFX_G_PRIME_HOLD_NS;
			busy = (upct >= RFX_G_PRIME_BUSY_ENTER_PCT) ||
			       (p->prime_busy_hold_ns &&
				time < p->prime_busy_hold_ns) ||
			       time < (u64)atomic64_read(&rfx_gaming_warmup_end_ns);

			fl = busy ? rfx_pct(fmax, RFX_G_PRIME_FLOOR_PCT) :
				    rfx_pct(fmax, RFX_G_PRIME_IDLE_FLOOR_PCT);
			cap = rfx_pct(fmax, RFX_G_PRIME_CAP_PCT);

			/*
			 * Only lift on a frame boost if Prime is actually BUSY. The
			 * render thread on this device lives mostly on Big and leaves
			 * Prime idle; lifting an idle Prime to the frame floor (~92%)
			 * during a sag/jank is pure waste heat on a core doing no
			 * render work. Cold-start is unaffected: the moment the render
			 * thread lands on Prime it is busy (util / hold / warmup), so
			 * it still gets the lift exactly when it matters.
			 */
			if (fboost && busy)
				fl = max(fl, rfx_pct(fmax, RFX_G_PRIME_FRAME_PCT));
			if (freq < fl)
				freq = fl;
			if (freq > cap)
				freq = cap;
		} else if (!little) {		/* Big: carries most load */
			unsigned int fl = rfx_pct(fmax, RFX_G_BIG_FLOOR_PCT);
			unsigned int cap = rfx_pct(fmax, RFX_G_BIG_CAP_PCT);

			if (fboost)
				fl = max(fl, rfx_pct(fmax, RFX_G_BIG_FRAME_PCT));
			if (freq < fl)
				freq = fl;
			if (freq > cap)
				freq = cap;
		} else {			/* Little: dynamic, soft floor */
			unsigned int cap = rfx_pct(fmax, RFX_G_LITTLE_CAP_PCT);
			unsigned int fl = 0;
			bool busy;

			/* Busy-hold: bridge micro-idle so we don't crash to fmin. */
			if (upct > RFX_G_LITTLE_FLOOR_ENTER_PCT)
				p->little_busy_hold_ns = time + RFX_G_LITTLE_HOLD_NS;
			busy = (upct > RFX_G_LITTLE_FLOOR_ENTER_PCT) ||
			       (p->little_busy_hold_ns &&
				time < p->little_busy_hold_ns);

			if (busy)
				fl = rfx_pct(fmax, RFX_G_LITTLE_FLOOR_PCT);
			/* Same rule as Prime: don't lift an idle Little (waste heat). */
			if (fboost && busy)
				fl = max(fl, rfx_pct(fmax, RFX_G_LITTLE_FRAME_PCT));
			if (freq < fl)
				freq = fl;
			if (freq > cap)
				freq = cap;
		}
	} else {
		bool ui_active;

		/*
		 * Detect a render burst: a sharp rise in smoothed util re-arms
		 * the UI floor. Catches caption draws / open-close animations /
		 * fling-scrolls that touch detection alone would miss.
		 */
		if (upct > p->prev_upct &&
		    upct - p->prev_upct >= RFX_D_RAMP_DELTA_PCT)
			p->ui_boost_end_ns = time + RFX_D_UI_BOOST_NS;
		p->prev_upct = upct;

		ui_active = rfx_input_active(time) ||
			    (p->ui_boost_end_ns && time < p->ui_boost_end_ns);

		if (little) {
			unsigned int cap = ui_active ?
				rfx_pct(fmax, RFX_D_LITTLE_BOOST_CAP_PCT) :
				rfx_pct(fmax, RFX_D_LITTLE_CAP_PCT);

			if (freq > cap)
				freq = cap;
			if (ui_active) {
				unsigned int fl = rfx_pct(fmax, RFX_D_LITTLE_UI_FLOOR_PCT);

				if (freq < fl)
					freq = fl;
			}
		} else if (!prime && ui_active) {	/* Big UI floor */
			unsigned int fl = rfx_pct(fmax, RFX_D_BIG_UI_FLOOR_PCT);

			if (freq < fl)
				freq = fl;
		}
	}

	freq = rfx_thermal_clamp(p, freq, fmax, time);
	freq = clamp(freq, fmin, fmax);

	/*
	 * Publish each perf cluster's headroom for the frame-boost gate. The
	 * render thread can live on Big OR Prime, so the gate needs both: it
	 * arms when the least-loaded perf cluster still has room (see
	 * rfx_frame_account). Little is support work, not a gate input.
	 */
	if (gaming) {
		unsigned int pct = (unsigned int)((u64)freq * 100 / fmax);

		if (prime)
			atomic_set(&rfx_prime_perf_pct, pct);
		else if (!little)
			atomic_set(&rfx_sys_perf_pct, pct);
	}

	if (freq == p->cached_raw_freq && !p->need_freq_update)
		return p->next_freq;
	p->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(pol, freq);
}

/* ===================================================================== */
/* IO-wait boost (unchanged behaviour from schedutil lineage)            */
/* ===================================================================== */

static bool rfx_iowait_reset(struct rfx_cpu *rfx_c, u64 time, bool set)
{
	s64 delta_ns = time - rfx_c->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	rfx_c->iowait_boost = set ? IOWAIT_BOOST_MIN : 0;
	rfx_c->iowait_boost_pending = set;
	return true;
}

static void rfx_iowait_boost(struct rfx_cpu *rfx_c, u64 time, unsigned int flags)
{
	bool set = flags & SCHED_CPUFREQ_IOWAIT;
	unsigned long max_cap;
	unsigned int cap;

	/* Reset the boost if the CPU appears to have been idle long enough. */
	if (rfx_c->iowait_boost && rfx_iowait_reset(rfx_c, time, set))
		return;

	/* Only tasks waking up after IO get boosted. */
	if (!set)
		return;

	/* Double the boost at most once per IO-wakeup request. */
	if (rfx_c->iowait_boost_pending)
		return;
	rfx_c->iowait_boost_pending = true;

	/*
	 * Per-cluster boost ceiling: LITTLE is held low (support work), the
	 * big/prime cluster may ramp to 3/4 capacity so a wake-from-IO burst
	 * (e.g. in-game asset streaming) reaches a useful OPP immediately,
	 * before PELT util catches up. Bounded, so it never pins the core.
	 */
	max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	cap = rfx_cap_is_little(max_cap) ? (SCHED_CAPACITY_SCALE / 6) :
					   (SCHED_CAPACITY_SCALE * 3 / 4);

	/* Double the existing boost, else start at the minimum. */
	if (rfx_c->iowait_boost)
		rfx_c->iowait_boost = min_t(unsigned int,
					    rfx_c->iowait_boost << 1, cap);
	else
		rfx_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long rfx_iowait_apply(struct rfx_cpu *rfx_c, u64 time,
				      unsigned long max_cap)
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
	return rfx_c->iowait_boost * max_cap >> SCHED_CAPACITY_SHIFT;
}

static void rfx_get_util(struct rfx_cpu *rfx_c, unsigned long boost)
{
	rfx_get_util_gki510(rfx_c->cpu, boost, &rfx_c->util, &rfx_c->bwmin);
}

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bwmin))
		rfx_c->rfx_policy->need_freq_update = true;
}

/* ===================================================================== */
/* Rate limiting                                                         */
/* ===================================================================== */

/* Set the active down-rate-limit for this update (long while gaming). */
static inline void rfx_set_down_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->down_rate_delay_ns = (s64)RFX_GAMING_DOWN_US * NSEC_PER_USEC;
	else
		p->down_rate_delay_ns =
			(s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
}

/* up-rate-limit: instant up while gaming, tunable otherwise. */
static inline void rfx_pol_up_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->up_rate_delay_ns = 0;
	else
		p->up_rate_delay_ns =
			(s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
}

/* Evaluation gate: cheap throttle on how often we recompute at all. */
static bool rfx_should_update_freq(struct rfx_policy *p, u64 time)
{
	s64 delta;

	if (!p || !p->policy)
		return false;
	if (!cpufreq_this_cpu_can_update(p->policy))
		return false;

	if (unlikely(READ_ONCE(p->limits_changed))) {
		WRITE_ONCE(p->limits_changed, false);
		p->need_freq_update = true;
		smp_mb();
		return true;
	}
	if (p->need_freq_update)
		return true;

	delta = (s64)(time - max(p->last_upfreq_time, p->last_downfreq_time));
	return delta >= p->freq_update_delay_ns;
}

/* Commit next_freq subject to directional up/down rate limits. */
static bool rfx_commit_freq(struct rfx_policy *p, u64 time, unsigned int next_freq)
{
	s64 delta;

	if (p->need_freq_update) {
		p->need_freq_update = false;
		if (p->next_freq == next_freq)
			return false;
	} else if (p->next_freq == next_freq) {
		return false;
	}

	if (next_freq < p->next_freq) {
		delta = (s64)(time - p->last_downfreq_time);
		if (p->down_rate_delay_ns > 0 && delta < p->down_rate_delay_ns) {
			/*
			 * Down-step deferred by the rate limiter. We must NOT
			 * leave the OPP cache pointing at this (lower) raw freq
			 * while next_freq still holds the old (higher) one: the
			 * cache short-circuit in rfx_target_freq would then keep
			 * returning the stale-high next_freq every update and the
			 * frequency would stay pinned high far longer than the
			 * intended rate-limit window (stuck-high -> wasted heat ->
			 * vendor throttle). Invalidate the cache so the next
			 * evaluation re-derives the true target and commits the
			 * down-step as soon as the window elapses.
			 */
			p->cached_raw_freq = 0;
			return false;
		}
		p->last_downfreq_time = time;
	} else {
		delta = (s64)(time - p->last_upfreq_time);
		if (p->up_rate_delay_ns > 0 && delta < p->up_rate_delay_ns) {
			p->cached_raw_freq = 0;
			return false;
		}
		p->last_upfreq_time = time;
	}

	p->next_freq = next_freq;
	return true;
}

/* ===================================================================== */
/* Update hooks                                                          */
/* ===================================================================== */

static void rfx_deferred_update(struct rfx_policy *p)
{
	if (!p->work_in_progress) {
		p->work_in_progress = true;
		irq_work_queue(&p->irq_work);
	}
}

static void rfx_update_single_freq(struct update_util_data *hook, u64 time,
				   unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *p = rfx_c->rfx_policy;
	bool gaming = rfx_gaming_enabled();
	unsigned long max_cap, boost, eff;
	unsigned int next_f;

	max_cap = arch_scale_cpu_capacity(rfx_c->cpu);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (!rfx_should_update_freq(p, time))
		return;

	boost = rfx_iowait_apply(rfx_c, time, max_cap);
	rfx_get_util(rfx_c, boost);
	eff = max(rfx_c->util, boost);
	rfx_c->filt_util = rfx_ema(rfx_c->filt_util, eff, gaming);

	if (gaming)
		rfx_frame_account(time);

	rfx_set_down_delay(p, gaming);
	rfx_pol_up_delay(p, gaming);

	next_f = rfx_target_freq(p, rfx_c->filt_util, max_cap, time, gaming);

	if (!rfx_commit_freq(p, time, next_f))
		return;

	if (p->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(p->policy, p->next_freq);
	} else {
		raw_spin_lock(&p->update_lock);
		rfx_deferred_update(p);
		raw_spin_unlock(&p->update_lock);
	}
}

static unsigned int rfx_next_freq_shared(struct rfx_cpu *rfx_c, u64 time,
					 bool gaming)
{
	struct rfx_policy *p = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	unsigned long max_util = 0;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *jc = per_cpu_ptr(&rfx_cpu, j);
		unsigned long jb, je;

		jb = rfx_iowait_apply(jc, time, max_cap);
		rfx_get_util(jc, jb);
		je = max(jc->util, jb);
		jc->filt_util = rfx_ema(jc->filt_util, je, gaming);
		if (jc->filt_util > max_util)
			max_util = jc->filt_util;
	}

	if (gaming)
		rfx_frame_account(time);

	rfx_set_down_delay(p, gaming);
	rfx_pol_up_delay(p, gaming);

	return rfx_target_freq(p, max_util, max_cap, time, gaming);
}

static void rfx_update_shared(struct update_util_data *hook, u64 time,
			      unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *p = rfx_c->rfx_policy;
	bool gaming = rfx_gaming_enabled();
	unsigned int next_f;

	raw_spin_lock(&p->update_lock);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (rfx_should_update_freq(p, time)) {
		next_f = rfx_next_freq_shared(rfx_c, time, gaming);
		if (rfx_commit_freq(p, time, next_f)) {
			if (p->policy->fast_switch_enabled)
				cpufreq_driver_fast_switch(p->policy, p->next_freq);
			else
				rfx_deferred_update(p);
		}
	}

	raw_spin_unlock(&p->update_lock);
}

static void rfx_work(struct kthread_work *work)
{
	struct rfx_policy *p = container_of(work, struct rfx_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&p->update_lock, flags);
	freq = p->next_freq;
	p->work_in_progress = false;
	raw_spin_unlock_irqrestore(&p->update_lock, flags);

	mutex_lock(&p->work_lock);
	cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&p->work_lock);
}

static void rfx_irq_work(struct irq_work *irq_work)
{
	struct rfx_policy *p = container_of(irq_work, struct rfx_policy, irq_work);

	kthread_queue_work(&p->worker, &p->work);
}

/* ===================================================================== */
/* Thermal poller (slow path, may sleep -> never in the util hook)       */
/* ===================================================================== */

#ifdef CONFIG_THERMAL
static struct thermal_zone_device *rfx_tz;
static char rfx_tz_name[THERMAL_NAME_LENGTH];
static bool rfx_tz_user_set;		/* a manual thermal_zone write wins */
static int rfx_tz_tries;		/* bound the auto-bind probing */
#define RFX_TZ_MAX_TRIES	60

/*
 * Curated SKIN / board sensor names for gaming auto-bind. NEVER list a
 * CPU-junction zone here - those read 50-70C in normal play and would throttle
 * mid-match (the v6.11 regression). On gaming_mode=1 the poller lazily binds the
 * first of these that exists, so heavy/long sessions get the smooth pre-emptive
 * thermal shave with no manual setup. If a device's skin zone is not in this
 * list, write its name (or "auto") to the thermal_zone sysfs instead.
 */
static const char * const rfx_skin_candidates[] = {
	/* Qualcomm / generic skin + board sensors. */
	"skin-therm", "quiet-therm", "skin-msm-therm",
	"xo-therm", "sys-therm", "board-therm",
	/*
	 * MediaTek board / skin NTC thermistors (skin-scale, ~38-48C). These
	 * are physical board sensors - NOT the cpu, gpu or soc junction zones
	 * (60-80C in play), which would throttle the skin curve mid-match.
	 * ap_ntc (NTC beside the AP) is MTK's standard skin sensor;
	 * backlight_therm (display NTC) is a close fallback.
	 */
	"ap_ntc", "backlight_therm", "mtkcsbts",
};

static void rfx_tz_autobind(void)
{
	int i;

	if (rfx_tz || rfx_tz_user_set || rfx_tz_tries >= RFX_TZ_MAX_TRIES)
		return;
	rfx_tz_tries++;

	for (i = 0; i < ARRAY_SIZE(rfx_skin_candidates); i++) {
		struct thermal_zone_device *tz =
			thermal_zone_get_zone_by_name(rfx_skin_candidates[i]);

		if (!IS_ERR(tz)) {
			strscpy(rfx_tz_name, rfx_skin_candidates[i],
				sizeof(rfx_tz_name));
			rfx_tz = tz;
			pr_info("vorpal: auto-bound skin thermal zone '%s'\n",
				rfx_tz_name);
			return;
		}
	}
}

/* Drop an auto-bound zone on gaming-off; a manual bind is kept. */
static void rfx_tz_release_auto(void)
{
	if (!rfx_tz_user_set) {
		rfx_tz = NULL;
		rfx_tz_name[0] = '\0';
		rfx_tz_tries = 0;
	}
}
#endif
static struct delayed_work rfx_thermal_work;
static u64 rfx_jank_window_start;
static int rfx_temp_smoothed = -1;	/* EMA of the sensor, -1 = unset */

static void rfx_thermal_fn(struct work_struct *w)
{
	int t_mc = 0;
	bool have = false;
	unsigned int delay_ms;
	int frames, janks;
	u64 now = ktime_get_ns();

#ifdef CONFIG_THERMAL
	struct thermal_zone_device *tz;

	/* Lazily bind a skin zone while gaming (rides out late driver reg). */
	if (rfx_gaming_enabled())
		rfx_tz_autobind();

	tz = rfx_tz;			/* snapshot: gaming-off may clear it */
	if (tz && !thermal_zone_get_temp(tz, &t_mc))
		have = true;
#endif
	if (!have) {
		t_mc = atomic_read(&rfx_temp_mc);
		if (t_mc > 0)
			have = true;
	}

	/*
	 * Smooth the raw sensor with an EMA (alpha 0.25) before mapping to a cap.
	 * A bare sensor dithers ~+/-0.5C; when it sits near a breakpoint that
	 * dither would flip the published cap (e.g. 100<->98) every poll, and the
	 * fast-path step controller would chase it up and down -> a sawtooth on
	 * the clamped freq = visible jitter right at the throttle knee. Filtering
	 * the temperature publishes a steady cap, so the throttle stays smooth.
	 */
	if (have) {
		if (rfx_temp_smoothed < 0)
			rfx_temp_smoothed = t_mc;
		else
			rfx_temp_smoothed += (t_mc - rfx_temp_smoothed) >> 2;
		atomic_set(&rfx_thermal_cap_pct, rfx_temp_to_cap(rfx_temp_smoothed));
	} else {
		rfx_temp_smoothed = -1;
		atomic_set(&rfx_thermal_cap_pct, 100);
	}

	/* Jank window: publish jank percent roughly every RFX_JANK_WINDOW_NS. */
	if (!rfx_jank_window_start)
		rfx_jank_window_start = now;
	if (now - rfx_jank_window_start >= RFX_JANK_WINDOW_NS) {
		frames = atomic_xchg(&rfx_frames_seen, 0);
		janks = atomic_xchg(&rfx_janks_seen, 0);
		atomic_set(&rfx_jank_pct, frames ? janks * 100 / frames : 0);
		rfx_jank_window_start = now;
	}

	delay_ms = rfx_gaming_enabled() ? RFX_THERMAL_POLL_GAMING_MS :
					  RFX_THERMAL_POLL_IDLE_MS;
	schedule_delayed_work(&rfx_thermal_work, msecs_to_jiffies(delay_ms));
}

/* ===================================================================== */
/* Input handler (daily touch boost; off during gaming)                  */
/* ===================================================================== */

static void rfx_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	if (rfx_gaming_enabled())
		return;
	if (type == EV_ABS || type == EV_KEY)
		atomic64_set(&rfx_input_ts_ns, ktime_get_ns());
}

static int rfx_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int err;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "vorpal";

	err = input_register_handle(handle);
	if (err)
		goto err_free;
	err = input_open_device(handle);
	if (err)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return err;
}

static void rfx_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id rfx_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
	},
	{ },
};

static struct input_handler rfx_input_handler = {
	.event		= rfx_input_event,
	.connect	= rfx_input_connect,
	.disconnect	= rfx_input_disconnect,
	.name		= "vorpal",
	.id_table	= rfx_input_ids,
};

/* ===================================================================== */
/* sysfs                                                                 */
/* ===================================================================== */

static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->rate_limit_us);
}
static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	struct rfx_policy *p;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->rate_limit_us = val;
	list_for_each_entry(p, &attr_set->policy_list, tunables_hook)
		p->freq_update_delay_ns = (s64)val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr rate_limit_us = __ATTR_RW(rate_limit_us);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->up_rate_limit_us);
}
static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->up_rate_limit_us = val;
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->down_rate_limit_us);
}
static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->down_rate_limit_us = val;
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

/* Reset transient gaming residue on every live policy (all clusters). */
static void rfx_reset_all_policies(void)
{
	struct rfx_policy *p;
	unsigned long flags;

	atomic64_set(&rfx_frame_boost_end_ns, 0);

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_for_each_entry(p, &rfx_policy_list, gov_node)
		p->need_freq_update = true;
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
}

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->gaming_mode);
}
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;

	t->gaming_mode = val;
	atomic_set(&rfx_gaming, val);
	/* Drive the scheduler-side gaming biases in lockstep (KMI-safe int). */
	WRITE_ONCE(sched_gaming_active, (int)val);

	if (!val) {
		atomic_set(&rfx_frame_time_us, 0);
		atomic_set(&rfx_frames_seen, 0);
		atomic_set(&rfx_janks_seen, 0);
		atomic_set(&rfx_jank_pct, 0);
		atomic_set(&rfx_ft_ema_us, 0);
		atomic64_set(&rfx_last_present_consumed, 0);
		atomic_set(&rfx_sys_perf_pct, 0);
		atomic_set(&rfx_prime_perf_pct, 0);
		rfx_reset_all_policies();
		atomic64_set(&rfx_gaming_warmup_end_ns, 0);
#ifdef CONFIG_THERMAL
		/* Unbind the auto-bound skin zone -> back to (none) for daily. */
		rfx_tz_release_auto();
#endif
	} else {
		/*
		 * Default the gaming frame budget to 90fps (the data-proven
		 * sustainable target on this device's thermal envelope) unless
		 * userspace has pinned a value via the frame_budget_us sysfs.
		 * NOTE: this is the governor's TARGET; pair it with an in-game /
		 * GFX-tool 90fps cap for the full thermal benefit.
		 */
		if (!rfx_frame_budget_user_set)
			atomic_set(&rfx_frame_budget_us, RFX_FRAME_BUDGET_US_GAMING);
		/* Fresh frame-pacing baseline so the first interval is sane. */
		atomic_set(&rfx_ft_ema_us, 0);
		atomic64_set(&rfx_last_present_consumed, 0);
		atomic_set(&rfx_sys_perf_pct, 0);
		atomic_set(&rfx_prime_perf_pct, 0);
		/* Pre-warm Prime so the first frames don't land on a cold core. */
		atomic64_set(&rfx_gaming_warmup_end_ns,
			     ktime_get_ns() + RFX_G_PRIME_WARMUP_NS);
#ifdef CONFIG_THERMAL
		/* Re-probe the skin zone for this session (driver may be late). */
		rfx_tz_tries = 0;
#endif
		/* Sample temperature sooner once gaming begins (also auto-binds). */
		mod_delayed_work(system_wq, &rfx_thermal_work,
				 msecs_to_jiffies(RFX_THERMAL_POLL_GAMING_MS));
	}
	return count;
}
static struct governor_attr gaming_mode = __ATTR_RW(gaming_mode);

static ssize_t temp_mc_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_temp_mc));
}
static ssize_t temp_mc_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&rfx_temp_mc, val);
	return count;
}
static struct governor_attr temp_mc = __ATTR_RW(temp_mc);

static ssize_t thermal_zone_show(struct gov_attr_set *attr_set, char *buf)
{
#ifdef CONFIG_THERMAL
	return sprintf(buf, "%s\n", rfx_tz_name[0] ? rfx_tz_name : "(none)");
#else
	return sprintf(buf, "(no CONFIG_THERMAL)\n");
#endif
}
static ssize_t thermal_zone_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
#ifdef CONFIG_THERMAL
	struct thermal_zone_device *tz;
	char name[THERMAL_NAME_LENGTH];

	strscpy(name, buf, sizeof(name));
	strim(name);

	/* "auto" hands control back to the gaming skin-zone auto-bind. */
	if (!strcmp(name, "auto")) {
		rfx_tz_user_set = false;
		rfx_tz = NULL;
		rfx_tz_name[0] = '\0';
		rfx_tz_tries = 0;
		return count;
	}

	tz = thermal_zone_get_zone_by_name(name);
	if (IS_ERR(tz))
		return -EINVAL;
	rfx_tz = tz;
	strscpy(rfx_tz_name, name, sizeof(rfx_tz_name));
	rfx_tz_user_set = true;		/* manual choice overrides auto-bind */
	return count;
#else
	return -ENODEV;
#endif
}
static struct governor_attr thermal_zone = __ATTR_RW(thermal_zone);

static ssize_t frame_budget_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_frame_budget_us));
}
static ssize_t frame_budget_us_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val < 1000)
		return -EINVAL;
	atomic_set(&rfx_frame_budget_us, val);
	rfx_frame_budget_user_set = true;	/* honour the manual pin over the gaming default */
	return count;
}
static struct governor_attr frame_budget_us = __ATTR_RW(frame_budget_us);

static ssize_t frame_time_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_frame_time_us));
}
static ssize_t frame_time_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&rfx_frame_time_us, val);
	return count;
}
static struct governor_attr frame_time_us = __ATTR_RW(frame_time_us);

static ssize_t jank_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_jank_pct));
}
static struct governor_attr jank_pct = __ATTR_RO(jank_pct);

/*
 * RO diagnostic: the thermal cap the governor is currently publishing (percent
 * of fmax). 100 = not throttling. If a cluster's freq sags while this reads
 * <100, the governor thermal step is the cause (raise the breakpoints / check
 * the bound zone scale); if it reads 100, the sag is util-following / placement.
 */
static ssize_t cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_thermal_cap_pct));
}
static struct governor_attr cap_pct = __ATTR_RO(cap_pct);

static struct attribute *rfx_little_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
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
	&temp_mc.attr,
	&thermal_zone.attr,
	&frame_budget_us.attr,
	&frame_time_us.attr,
	&jank_pct.attr,
	&cap_pct.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_prime);

static void rfx_tunables_free(struct kobject *kobj)
{
	kfree(to_rfx_tunables(rfx_to_gov_attr_set(kobj)));
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

/* ===================================================================== */
/* Allocation / kthread                                                  */
/* ===================================================================== */

static struct rfx_policy *rfx_policy_alloc(struct cpufreq_policy *policy)
{
	struct rfx_policy *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;
	p->policy = policy;
	p->thermal_applied_pct = 100;
	raw_spin_lock_init(&p->update_lock);
	INIT_LIST_HEAD(&p->gov_node);
	return p;
}

static void rfx_policy_free(struct rfx_policy *p)
{
	kfree(p);
}

static int rfx_kthread_create(struct rfx_policy *p)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = p->policy;
	struct sched_param sp = { .sched_priority = MAX_RT_PRIO / 2 };
	int ret;

	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&p->work, rfx_work);
	kthread_init_worker(&p->worker);
	thread = kthread_create(kthread_worker_fn, &p->worker, "rfx_gov/%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("vorpal: kthread create failed %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &sp);
	if (ret) {
		kthread_stop(thread);
		pr_warn("vorpal: failed to set SCHED_FIFO\n");
		return ret;
	}

	p->thread = thread;
	if (policy->dvfs_possible_from_any_cpu)
		set_cpus_allowed_ptr(thread, policy->related_cpus);
	else
		kthread_bind_mask(thread, policy->related_cpus);

	init_irq_work(&p->irq_work, rfx_irq_work);
	mutex_init(&p->work_lock);
	wake_up_process(thread);
	return 0;
}

static void rfx_kthread_stop(struct rfx_policy *p)
{
	if (p->policy->fast_switch_enabled)
		return;
	kthread_flush_worker(&p->worker);
	kthread_stop(p->thread);
	mutex_destroy(&p->work_lock);
}

static struct rfx_tunables *rfx_tunables_alloc(struct rfx_policy *p)
{
	struct rfx_tunables *t;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (t) {
		gov_attr_set_init(&t->attr_set, &p->tunables_hook);
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

/* ===================================================================== */
/* Governor callbacks                                                    */
/* ===================================================================== */

static int rfx_init(struct cpufreq_policy *policy)
{
	struct rfx_policy *p;
	struct rfx_tunables *t;
	unsigned long max_cap;
	struct kobj_type *ktype;
	int ret = 0;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	p = rfx_policy_alloc(policy);
	if (!p) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = rfx_kthread_create(p);
	if (ret)
		goto free_p;

	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	p->is_prime = rfx_cap_is_prime(max_cap);
	p->is_little = rfx_cap_is_little(max_cap);

	mutex_lock(&rfx_global_tunables_lock);

	if (rfx_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = p;
		p->tunables = rfx_global_tunables;
		gov_attr_set_get(&rfx_global_tunables->attr_set, &p->tunables_hook);
		goto out;
	}

	t = rfx_tunables_alloc(p);
	if (!t) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	if (p->is_little) {
		t->cluster_type = RFX_CLUSTER_LITTLE;
		t->rate_limit_us = RFX_LITTLE_RATE_US;
		t->up_rate_limit_us = RFX_LITTLE_UP_US;
		t->down_rate_limit_us = RFX_LITTLE_DOWN_US;
		ktype = &rfx_little_ktype;
	} else if (p->is_prime) {
		t->cluster_type = RFX_CLUSTER_PRIME;
		t->rate_limit_us = RFX_PRIME_RATE_US;
		t->up_rate_limit_us = RFX_PRIME_UP_US;
		t->down_rate_limit_us = RFX_PRIME_DOWN_US;
		ktype = &rfx_prime_ktype;
	} else {
		t->cluster_type = RFX_CLUSTER_BIG;
		t->rate_limit_us = RFX_BIG_RATE_US;
		t->up_rate_limit_us = RFX_BIG_UP_US;
		t->down_rate_limit_us = RFX_BIG_DOWN_US;
		ktype = &rfx_big_ktype;
	}

	policy->governor_data = p;
	p->tunables = t;

	ret = kobject_init_and_add(&t->attr_set.kobj, ktype,
				   get_governor_parent_kobj(policy),
				   "%s", vorpal_gov.name);
	if (ret)
		goto fail;

out:
	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
	mutex_unlock(&rfx_global_tunables_lock);
	return 0;

fail:
	kobject_put(&t->attr_set.kobj);
	policy->governor_data = NULL;
	rfx_clear_global_tunables();
stop_kthread:
	rfx_kthread_stop(p);
	mutex_unlock(&rfx_global_tunables_lock);
free_p:
	rfx_policy_free(p);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("vorpal: init failed error %d\n", ret);
	return ret;
}

static void rfx_exit(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	struct rfx_tunables *t = p->tunables;
	unsigned int count;

	mutex_lock(&rfx_global_tunables_lock);
	count = gov_attr_set_put(&t->attr_set, &p->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		rfx_clear_global_tunables();
	mutex_unlock(&rfx_global_tunables_lock);

	rfx_kthread_stop(p);
	rfx_policy_free(p);
	cpufreq_disable_fast_switch(policy);
}

static int rfx_start(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned long flags;
	unsigned int cpu;
	u64 now = ktime_get_ns();

	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;

	p->last_upfreq_time = now;
	p->last_downfreq_time = now;
	p->next_freq = policy->cur > 0 ? policy->cur : policy->cpuinfo.min_freq;
	p->cached_raw_freq = 0;
	p->work_in_progress = false;
	p->limits_changed = false;
	p->need_freq_update = false;
	p->prev_upct = 0;
	p->ui_boost_end_ns = 0;
	p->prime_busy_hold_ns = 0;
	p->little_busy_hold_ns = 0;
	p->thermal_applied_pct = 100;
	p->thermal_step_ns = now;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_add(&p->gov_node, &rfx_policy_list);
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = per_cpu_ptr(&rfx_cpu, cpu);

		memset(rfx_c, 0, sizeof(*rfx_c));
		rfx_c->cpu = cpu;
		rfx_c->rfx_policy = p;
	}

	uu = policy_is_shared(policy) ? rfx_update_shared : rfx_update_single_freq;
	for_each_cpu(cpu, policy->cpus)
		cpufreq_add_update_util_hook(cpu, &per_cpu_ptr(&rfx_cpu, cpu)->update_util, uu);
	return 0;
}

static void rfx_stop(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	unsigned long flags;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_del(&p->gov_node);
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&p->irq_work);
		kthread_cancel_work_sync(&p->work);
	}
}

static void rfx_limits(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&p->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&p->work_lock);
	}
	smp_wmb();
	WRITE_ONCE(p->limits_changed, true);
}

static struct cpufreq_governor vorpal_gov = {
	.name = CPUFREQ_VORPAL_NAME,
	.owner = THIS_MODULE,
	.flags = CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init = rfx_init,
	.exit = rfx_exit,
	.start = rfx_start,
	.stop = rfx_stop,
	.limits = rfx_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_VORPAL
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &vorpal_gov;
}
#endif

static int __init vorpal_gov_init(void)
{
	int ret;

	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION,
		CPUFREQ_VORPAL_AUTHOR);

	INIT_DEFERRABLE_WORK(&rfx_thermal_work, rfx_thermal_fn);
	schedule_delayed_work(&rfx_thermal_work,
			      msecs_to_jiffies(RFX_THERMAL_POLL_IDLE_MS));

	if (input_register_handler(&rfx_input_handler))
		pr_warn("vorpal: input handler register failed (touch boost off)\n");

	ret = cpufreq_register_governor(&vorpal_gov);
	if (ret) {
		input_unregister_handler(&rfx_input_handler);
		cancel_delayed_work_sync(&rfx_thermal_work);
	}
	return ret;
}

static void __exit vorpal_gov_exit(void)
{
	cpufreq_unregister_governor(&vorpal_gov);
	input_unregister_handler(&rfx_input_handler);
	cancel_delayed_work_sync(&rfx_thermal_work);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR(CPUFREQ_VORPAL_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.0");
