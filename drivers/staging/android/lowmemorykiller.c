/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/profile.h>
#include <linux/notifier.h>
#include <linux/freezer.h>
#include <linux/ratelimit.h>

//#define MTK_LMK_USER_EVENT

#ifdef MTK_LMK_USER_EVENT
#include <linux/miscdevice.h>
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
#include <mt-plat/aee.h>
#include <disp_assert_layer.h>
#endif

#include "internal.h"
#ifdef CONFIG_HSWAP
#include <linux/delay.h>
#include <linux/kthread.h>
#include "../../block/zram/zram_drv.h"
#endif

#ifdef CONFIG_HIGHMEM
#define _ZONE ZONE_HIGHMEM
#else
#define _ZONE ZONE_NORMAL
#endif

#define CREATE_TRACE_POINTS
#include "trace/lowmemorykiller.h"

static DEFINE_SPINLOCK(lowmem_shrink_lock);
static short lowmem_warn_adj, lowmem_no_warn_adj = 0;
static u32 lowmem_debug_level = 1;
static short lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};

static int lowmem_adj_size = 4;
static int lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};

static int lowmem_minfree_size = 4;
static int lmk_fast_run = 0;

static int lmk_kill_cnt = 0;
#ifdef CONFIG_HSWAP
static int lmk_reclaim_cnt = 0;

enum alloc_pressure {
	PRESSURE_NORMAL,
	PRESSURE_HIGH
};

enum {
	KILL_LMK,
	KILL_MEMORY_PRESSURE,
	KILL_NO_RECLAIMABLE,
	KILL_RECLAIMING,
	KILL_SWAP_FULL,
	REASON_COUNT
};

static char* kill_reason_str[REASON_COUNT] = {
	"by lmk",
	"by mem pressure",
	"by no reclaimable",
	"by reclaming",
	"by swap full"
};
#endif

static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

static unsigned long lowmem_count(struct shrinker *s,
				  struct shrink_control *sc)
{
#ifdef CONFIG_FREEZER
	/* Don't bother LMK when system is freezing */
	if (pm_freezing)
		return 0;
#endif
	return global_node_page_state(NR_ACTIVE_ANON) +
		global_node_page_state(NR_ACTIVE_FILE) +
		global_node_page_state(NR_INACTIVE_ANON) +
		global_node_page_state(NR_INACTIVE_FILE);
}

#ifdef MTK_LMK_USER_EVENT
static const struct file_operations mtklmk_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice mtklmk_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mtklmk",
	.fops = &mtklmk_fops,
};

static struct work_struct mtklmk_work;
static int uevent_adj, uevent_minfree;
static void mtklmk_async_uevent(struct work_struct *work)
{
#define MTKLMK_EVENT_LENGTH	(24)
	char adj[MTKLMK_EVENT_LENGTH], free[MTKLMK_EVENT_LENGTH];
	char *envp[3] = { adj, free, NULL };

	snprintf(adj, MTKLMK_EVENT_LENGTH, "OOM_SCORE_ADJ=%d", uevent_adj);
	snprintf(free, MTKLMK_EVENT_LENGTH, "MINFREE=%d", uevent_minfree);
	kobject_uevent_env(&mtklmk_misc.this_device->kobj, KOBJ_CHANGE, envp);
#undef MTKLMK_EVENT_LENGTH
}

static unsigned int mtklmk_initialized;
static unsigned int mtklmk_uevent_timeout = 10000; /* ms */
module_param_named(uevent_timeout, mtklmk_uevent_timeout, uint, 0644);
static void mtklmk_uevent(int oom_score_adj, int minfree)
{
	static unsigned long last_time;
	unsigned long timeout;

	/* change to use jiffies */
	timeout = msecs_to_jiffies(mtklmk_uevent_timeout);

	if (!last_time)
		last_time = jiffies - timeout;

	if (time_before(jiffies, last_time + timeout))
		return;

	last_time = jiffies;

	uevent_adj = oom_score_adj;
	uevent_minfree = minfree;
	schedule_work(&mtklmk_work);
}
#endif

#ifdef CONFIG_MTK_ENABLE_AGO
/* Check memory status by zone, pgdat */
static int lowmem_check_status_by_zone(enum zone_type high_zoneidx,
				       int *other_free, int *other_file)
{
	struct pglist_data *pgdat;
	struct zone *z;
	enum zone_type zoneidx;
	unsigned long accumulated_pages = 0;
	u64 scale = (u64)totalram_pages;
	int new_other_free = 0, new_other_file = 0;
	int memory_pressure = 0;
	int unreclaimable = 0;

	if (high_zoneidx < MAX_NR_ZONES - 1) {
		/* Go through all memory nodes */
		for_each_online_pgdat(pgdat) {
			for (zoneidx = 0; zoneidx <= high_zoneidx; zoneidx++) {
				z = pgdat->node_zones + zoneidx;
				accumulated_pages += z->managed_pages;
				new_other_free +=
					zone_page_state(z, NR_FREE_PAGES);
				new_other_free -= high_wmark_pages(z);
				new_other_file +=
				zone_page_state(z, NR_ZONE_ACTIVE_FILE) +
				zone_page_state(z, NR_ZONE_INACTIVE_FILE);

				/* Compute memory pressure level */
				memory_pressure +=
				zone_page_state(z, NR_ZONE_ACTIVE_FILE) +
				zone_page_state(z, NR_ZONE_INACTIVE_FILE) +
#ifdef CONFIG_SWAP
				zone_page_state(z, NR_ZONE_ACTIVE_ANON) +
				zone_page_state(z, NR_ZONE_INACTIVE_ANON) +
#endif
				new_other_free;
			}

			/*
			 * Consider pgdat as unreclaimable when hitting one of
			 * following two cases,
			 * 1. Memory node is unreclaimable in vmscan.c
			 * 2. Memory node is reclaimable, but nearly no user
			 *    pages(under high wmark)
			 */
			if (!pgdat_reclaimable(pgdat) ||
			    (pgdat_reclaimable(pgdat) && memory_pressure < 0))
				unreclaimable++;
		}

		/*
		 * Update if we go through ONLY lower zone(s) ACTUALLY
		 * and scale in totalram_pages
		 */
		if (totalram_pages > accumulated_pages) {
			do_div(scale, accumulated_pages);
			if ((u64)totalram_pages >
			    (u64)accumulated_pages * scale)
				scale += 1;
			new_other_free *= scale;
			new_other_file *= scale;
		}

		/*
		 * Update if not kswapd or
		 * "being kswapd and high memory pressure"
		 */
		if (!current_is_kswapd() ||
		    (current_is_kswapd() && memory_pressure < 0)) {
			*other_free = new_other_free;
			*other_file = new_other_file;
		}
	}

	return unreclaimable;
}

/* Aggressive Memory Reclaim(AMR) */
static short lowmem_amr_check(int *to_be_aggressive, int other_file)
{
#ifdef CONFIG_SWAP
#ifdef CONFIG_64BIT
#define ENABLE_AMR_RAMSIZE	(0x60000)	/* > 1.5GB */
#else
#define ENABLE_AMR_RAMSIZE	(0x40000)	/* > 1GB */
#endif
	unsigned long swap_pages = 0;
	short amr_adj = OOM_SCORE_ADJ_MAX + 1;
#ifndef CONFIG_MTK_GMO_RAM_OPTIMIZE
	int i;
#endif
	swap_pages = atomic_long_read(&nr_swap_pages);
	/* More than 1/2 swap usage */
	if (swap_pages * 2 < total_swap_pages)
		(*to_be_aggressive)++;
	/* More than 3/4 swap usage */
	if (swap_pages * 4 < total_swap_pages)
		(*to_be_aggressive)++;

#ifndef CONFIG_MTK_GMO_RAM_OPTIMIZE
	/* Try to enable AMR when we have enough memory */
	if (totalram_pages < ENABLE_AMR_RAMSIZE) {
		*to_be_aggressive = 0;
	} else {
		i = lowmem_adj_size - 1;
		/*
		 * Comparing other_file with lowmem_minfree to make
		 * amr less aggressive.
		 * ex.
		 * For lowmem_adj[] = {0, 100, 200, 300, 900, 906},
		 * if swap usage > 50%,
		 * try to kill 906       when other_file >= lowmem_minfree[5]
		 * try to kill 300 ~ 906 when other_file  < lowmem_minfree[5]
		 */
		if (*to_be_aggressive > 0) {
			if (other_file < lowmem_minfree[i])
				i -= *to_be_aggressive;
			if (likely(i >= 0))
				amr_adj = lowmem_adj[i];
		}
	}
#endif

	return amr_adj;
#undef ENABLE_AMR_RAMSIZE

#else	/* !CONFIG_SWAP */
	*to_be_aggressive = 0;
	return OOM_SCORE_ADJ_MAX + 1;
#endif
}
#else
static int lowmem_check_status_by_zone(enum zone_type high_zoneidx,
				       int *other_free, int *other_file)
{
	return 0;
}

#define lowmem_amr_check(a, b) (short)(OOM_SCORE_ADJ_MAX + 1)
#endif

static void __lowmem_trigger_warning(struct task_struct *selected)
{
#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
#define MSG_SIZE_TO_AEE 70
	char msg_to_aee[MSG_SIZE_TO_AEE];

	lowmem_print(1, "low memory trigger kernel warning\n");
	snprintf(msg_to_aee, MSG_SIZE_TO_AEE,
		 "please contact AP/AF memory module owner[pid:%d]\n",
		 selected->pid);

	aee_kernel_warning_api("LMK", 0, DB_OPT_DEFAULT |
			       DB_OPT_DUMPSYS_ACTIVITY |
			       DB_OPT_LOW_MEMORY_KILLER |
			       DB_OPT_PID_MEMORY_INFO | /* smaps and hprof*/
			       DB_OPT_PROCESS_COREDUMP |
			       DB_OPT_DUMPSYS_SURFACEFLINGER |
			       DB_OPT_DUMPSYS_GFXINFO |
			       DB_OPT_DUMPSYS_PROCSTATS,
			       "Framework low memory\nCRDISPATCH_KEY:FLM_APAF",
			       msg_to_aee);
#undef MSG_SIZE_TO_AEE
#else
	pr_info("(%s) no warning triggered for selected(%s)(%d)\n",
		__func__, selected->comm, selected->pid);
#endif
}

/* try to trigger warning to get more information */
static void lowmem_trigger_warning(struct task_struct *selected,
				   short selected_oom_score_adj)
{
	static DEFINE_RATELIMIT_STATE(ratelimit, 60 * HZ, 1);

	if (selected_oom_score_adj > lowmem_warn_adj)
		return;

	if (!__ratelimit(&ratelimit))
		return;

	__lowmem_trigger_warning(selected);
}

/* try to dump more memory status */
static void dump_memory_status(short selected_oom_score_adj)
{
	static DEFINE_RATELIMIT_STATE(ratelimit, 5 * HZ, 1);
	static DEFINE_RATELIMIT_STATE(ratelimit_urgent, 2 * HZ, 1);

	if (selected_oom_score_adj > lowmem_warn_adj &&
	    !__ratelimit(&ratelimit))
		return;

	if (!__ratelimit(&ratelimit_urgent))
		return;

	show_task_mem();
	show_free_areas(0);
	oom_dump_extra_info();
}

#ifdef CONFIG_HSWAP
static int test_task_flag(struct task_struct *p, int flag)
{
	struct task_struct *t;

	for_each_thread(p, t) {
		task_lock(t);
		if (test_tsk_thread_flag(t, flag)) {
			task_unlock(t);
			return 1;
		}
		task_unlock(t);
	}

	return 0;
}

static bool reclaim_task_is_ok(int selected_task_anon_size)
{
	int free_size = zram0_free_size() - get_lowest_prio_swapper_space_nrpages();

	if (selected_task_anon_size < free_size)
		return true;

	return false;
}

#define OOM_SCORE_SERVICE_B_ADJ 800
#define OOM_SCORE_CACHED_APP_MIN_ADJ 900

static DEFINE_MUTEX(reclaim_mutex);

static struct completion reclaim_completion;
static struct task_struct *selected_task;

#define RESET_TIME 3600000 /* activity top time reset time(msec) */
static int reset_task_time_thread(void *p)
{
	struct task_struct *tsk;

	while (1) {
		struct task_struct *p;

		rcu_read_lock();
		for_each_process(tsk) {
			if (tsk->flags & PF_KTHREAD)
				continue;

			/* if task no longer has any memory ignore it */
			if (test_task_flag(tsk, TIF_MEMDIE))
				continue;

			if (tsk->exit_state || !tsk->mm)
				continue;

			p = find_lock_task_mm(tsk);
			if (!p)
				continue;

			if (p->signal->top_time)
				p->signal->top_time =
					(p->signal->top_time * 3) / 4;

			task_unlock(p);
		}
		rcu_read_unlock();
		msleep(RESET_TIME);
	}
	return 0;
}

static int reclaim_task_thread(void *p)
{
	int selected_tasksize;
	int efficiency;
	struct reclaim_param rp;

	init_completion(&reclaim_completion);

	while (1) {
		wait_for_completion(&reclaim_completion);

		mutex_lock(&reclaim_mutex);
		if (!selected_task)
			goto reclaim_end;

		lowmem_print(3, "hswap: scheduled reclaim task '%s'(%d), adj%hd\n",
				selected_task->comm, selected_task->pid,
				selected_task->signal->oom_score_adj);

		task_lock(selected_task);
		if (selected_task->exit_state || !selected_task->mm) {
			task_unlock(selected_task);
			put_task_struct(selected_task);
			goto reclaim_end;
		}

		selected_tasksize = get_mm_rss(selected_task->mm);
		if (!selected_tasksize) {
			task_unlock(selected_task);
			put_task_struct(selected_task);
			goto reclaim_end;
		}
		efficiency = selected_task->signal->reclaim_efficiency;
		task_unlock(selected_task);

		rp = reclaim_task_anon(selected_task, selected_tasksize);
		lowmem_print(3, "Reclaimed '%s' (%d), adj %hd,\n" \
				"   nr_reclaimed %d\n",
			     selected_task->comm, selected_task->pid,
			     selected_task->signal->oom_score_adj,
			     rp.nr_reclaimed);
		++lmk_reclaim_cnt;
		if (efficiency)
			efficiency = (efficiency + (rp.nr_reclaimed * 100) / selected_tasksize) / 2;
		else
			efficiency = (rp.nr_reclaimed * 100) / selected_tasksize;
		lowmem_print(3, "Reclaimed efficiency(%s, %d, %d) = %d\n",
				selected_task->comm,
				selected_tasksize,
				rp.nr_reclaimed,
				efficiency);
		selected_task->signal->reclaim_efficiency = efficiency;

		put_task_struct(selected_task);

reclaim_end:
		selected_task = NULL;

		init_completion(&reclaim_completion);
		mutex_unlock(&reclaim_mutex);
	}

	return 0;
}

#define SHRINK_TASK_MAX_CNT 100
#define LOOKING_SERVICE_MAX_CNT 5
struct task_struct* shrink_task[SHRINK_TASK_MAX_CNT];
char killed_task_comm[LOOKING_SERVICE_MAX_CNT][TASK_COMM_LEN];
char pre_killed_task_comm[TASK_COMM_LEN];
static int looking_service_cnt = 0;

struct sorted_task {
	struct task_struct *tp;
	int score;
	int tasksize;
	struct list_head list;
};

struct sorted_task st_by_time[SHRINK_TASK_MAX_CNT];
struct sorted_task st_by_count[SHRINK_TASK_MAX_CNT];
struct sorted_task st_by_memory[SHRINK_TASK_MAX_CNT];

struct list_head stl_by_time;
struct list_head stl_by_count;
struct list_head stl_by_memory;

struct task_struct *calc_hswap_kill_score(int shrink_task_cnt, int *rss_size)
{
	int i, j, k;
	struct sorted_task *cursor;
	struct sorted_task victim_task;
	int is_inserted;
	int high_frequent_kill_task = 0;
	int already_checked = 0;
	unsigned long tasksize;

	INIT_LIST_HEAD(&stl_by_time);
	INIT_LIST_HEAD(&stl_by_count);
	INIT_LIST_HEAD(&stl_by_memory);

	for (i = 0, j = 0; i < shrink_task_cnt; i++) {
		struct sorted_task *stp_by_time;
		struct sorted_task *stp_by_count;
		struct sorted_task *stp_by_memory;
		struct task_struct *task = shrink_task[i];

		task_lock(task);
		if (task->signal->oom_score_adj <= OOM_SCORE_CACHED_APP_MIN_ADJ) {
			if (already_checked || strncmp(task->comm, "earchbox:search", 15) != 0) {
				task_unlock(task);
				continue;
			} else {
				already_checked = 1;
			}
		}

		if (strncmp(task->comm, "dboxed_process0", 15) != 0) {
			if (pre_killed_task_comm[0]) {
				if (!strcmp(pre_killed_task_comm, task->comm)) {
					strcpy(killed_task_comm[looking_service_cnt], task->comm);
					looking_service_cnt = (looking_service_cnt + 1) % LOOKING_SERVICE_MAX_CNT;
					task_unlock(task);
					continue;
				}
			}

			for (k = 0; k < LOOKING_SERVICE_MAX_CNT; k++) {
				if (killed_task_comm[k][0]) {
					if (!strcmp(killed_task_comm[k], task->comm)) {
						high_frequent_kill_task = 1;
						break;
					}
				}
			}
		}

		if (high_frequent_kill_task) {
			lowmem_print(3, "%s: skip high frequent_kill task %s \n", __func__, task->comm);
			high_frequent_kill_task = 0;
			task_unlock(task);
			continue;
		}

		if (task->exit_state || !task->mm) {
			task_unlock(task);
			continue;
		}

		tasksize = get_mm_rss(task->mm);
		if (task->signal->oom_score_adj == OOM_SCORE_CACHED_APP_MIN_ADJ &&
				tasksize < 51200) {
			task_unlock(task);
			continue;
		}
		stp_by_time = &st_by_time[j];
		stp_by_count = &st_by_count[j];
		stp_by_memory = &st_by_memory[j];
		j++;
		INIT_LIST_HEAD(&stp_by_time->list);
		INIT_LIST_HEAD(&stp_by_count->list);
		INIT_LIST_HEAD(&stp_by_memory->list);

		stp_by_time->tp = task;
		stp_by_count->tp = task;
		stp_by_memory->tp = task;
		stp_by_time->score = 0;
		stp_by_count->score = 0;
		stp_by_memory->score = 0;
		stp_by_time->tasksize = tasksize;
		stp_by_count->tasksize = tasksize;
		stp_by_memory->tasksize = tasksize;
		if (list_empty(&stl_by_time) && list_empty(&stl_by_count)
				&& list_empty(&stl_by_memory)) {
			list_add(&stp_by_time->list, &stl_by_time);
			list_add(&stp_by_count->list, &stl_by_count);
			list_add(&stp_by_memory->list, &stl_by_memory);
			task_unlock(task);
			continue;
		}

		is_inserted = 0;
		list_for_each_entry(cursor, &stl_by_time, list) {
			if (stp_by_time->tp->signal->top_time <= cursor->tp->signal->top_time) {
				if (!is_inserted) {
					stp_by_time->score = cursor->score;
					list_add(&stp_by_time->list, cursor->list.prev);
					is_inserted = 1;
				}

				if (stp_by_time->tp->signal->top_time == cursor->tp->signal->top_time)
					break;

				cursor->score++;
			}
			if (list_is_last(&cursor->list, &stl_by_time)) {
				if (!is_inserted) {
					stp_by_time->score = cursor->score + 1;
					list_add(&stp_by_time->list, &cursor->list);
				}
				break;
			}
		}

		is_inserted = 0;
		list_for_each_entry(cursor, &stl_by_count, list) {
			if (stp_by_count->tp->signal->top_count <= cursor->tp->signal->top_count) {
				if (!is_inserted) {
					stp_by_count->score = cursor->score;
					list_add(&stp_by_count->list, cursor->list.prev);
					is_inserted = 1;
				}
				if (stp_by_count->tp->signal->top_count == cursor->tp->signal->top_count)
					break;

				cursor->score++;
			}

			if (list_is_last(&cursor->list, &stl_by_count)) {
				if (!is_inserted) {
					stp_by_count->score = cursor->score + 1;
					list_add(&stp_by_count->list, &cursor->list);
				}
				break;
			}
		}

		is_inserted = 0;
		list_for_each_entry(cursor, &stl_by_memory, list) {
			if (stp_by_memory->tasksize >= cursor->tasksize) {
				if (!is_inserted) {
					stp_by_memory->score = cursor->score;
					list_add(&stp_by_memory->list, cursor->list.prev);
					is_inserted = 1;
				}
				if (stp_by_memory->tasksize == cursor->tasksize)
					break;

				cursor->score++;
			}

			if (list_is_last(&cursor->list, &stl_by_memory)) {
				if (!is_inserted) {
					stp_by_memory->score = cursor->score + 1;
					list_add(&stp_by_memory->list, &cursor->list);
				}
				break;
			}
		}

		task_unlock(task);
	}

	lowmem_print(3, "%s: targeting killing task count = %d\n", __func__, j);
	victim_task.tp = NULL;
	victim_task.score = 0;
	victim_task.tasksize = 0;

	list_for_each_entry(cursor, &stl_by_time, list) {
		trace_lowmemory_kill_task_list(cursor->tp, lmk_kill_cnt);
	}

	for (i = 0 ; i < LOOKING_SERVICE_MAX_CNT; i++) {
		if (killed_task_comm[i][0])
			lowmem_print(3, "%s: abnormal service %s\n", __func__, killed_task_comm[i]);
	}

	while (!list_empty(&stl_by_time)) {
		struct sorted_task *cursor_other;
		struct sorted_task comp_task;
		cursor = list_first_entry(&stl_by_time, struct sorted_task, list);
		list_del(&cursor->list);
		comp_task.tp = NULL;
		comp_task.score = cursor->score;
		comp_task.tasksize = cursor->tasksize;
		list_for_each_entry(cursor_other, &stl_by_count, list) {
			if (cursor->tp->pid == cursor_other->tp->pid) {
				list_del(&cursor_other->list);
				comp_task.tp = cursor_other->tp;
				comp_task.score += cursor_other->score;
				break;
			}
		}

		list_for_each_entry(cursor_other, &stl_by_memory, list) {
			if (cursor->tp->pid == cursor_other->tp->pid) {
				list_del(&cursor_other->list);
				comp_task.tp = cursor_other->tp;
				comp_task.score += cursor_other->score;
				break;
			}
		}

		if (comp_task.tp == NULL)
			BUG();

		if (victim_task.tp == NULL) {
			victim_task.tp = comp_task.tp;
			victim_task.score = comp_task.score;
			victim_task.tasksize = comp_task.tasksize;
			continue;
		}

		if (comp_task.score < victim_task.score) {
			victim_task.tp = comp_task.tp;
			victim_task.score = comp_task.score;
			victim_task.tasksize = comp_task.tasksize;
		} else if (comp_task.score == victim_task.score) {
			if (comp_task.tp->signal->top_time <
					victim_task.tp->signal->top_time) {
				victim_task.tp = comp_task.tp;
				victim_task.tasksize = comp_task.tasksize;
			}
		}
	}

	*rss_size = victim_task.tasksize;
	return victim_task.tp;
}


static struct task_struct *find_suitable_reclaim(int shrink_task_cnt,
		int *rss_size)
{
	struct task_struct *selected = NULL;
	int selected_tasksize = 0;
	int tasksize, anonsize;
	long selected_top_time = -1;
	int i = 0;
	int efficiency = 0;

	for (i = 0; i < shrink_task_cnt; i++) {
		struct task_struct *p;

		p = shrink_task[i];

		task_lock(p);
		if (p->exit_state || !p->mm || p->signal->reclaimed) {
			task_unlock(p);
			continue;
		}

		tasksize = get_mm_rss(p->mm);
		anonsize = get_mm_counter(p->mm, MM_ANONPAGES);
		efficiency = p->signal->reclaim_efficiency;
		task_unlock(p);

		if (!tasksize)
			continue;

		if (!reclaim_task_is_ok(anonsize))
			continue;

		if (efficiency && tasksize > 100)
			tasksize = (tasksize * efficiency) / 100;

		if (selected_tasksize > tasksize)
			continue;

		selected_top_time = p->signal->top_time;
		selected_tasksize = tasksize;
		selected = p;
	}

	*rss_size = selected_tasksize;

	return selected;
}

static struct task_struct *find_suitable_kill_task(int shrink_task_cnt,
		int *rss_size)
{
	struct task_struct *selected = NULL;

	selected = calc_hswap_kill_score(shrink_task_cnt, rss_size);
	if (selected) {
		task_lock(selected);
		if (!(selected->exit_state || !selected->mm)) {
			*rss_size += get_mm_counter(selected->mm, MM_SWAPENTS);
		}
		task_unlock(selected);
	}

	return selected;
}

static void reclaim_arr_free(int shrink_task_cnt)
{
	int i;

	for (i = 0; i < shrink_task_cnt; i++)
		shrink_task[i] = NULL;
}

static unsigned long before_called_ts = 0;
int is_first_latency = 1;
#define TIME_ARR_SIZE  100
static int time_arr_size = 3;
static long arr_ts[TIME_ARR_SIZE] = {0, };
static int ts_idx = 0;
static long avg_treshold = 100;

static long calc_ts_avg(long *arr_ts, int arr_size)
{
	long avg = 0;
	int i = 0;

	for (; i < arr_size; i++) {
		avg += arr_ts[i];
	}

	return (avg / arr_size);
}

static int reset_latency(void)
{
	int i = 0;

	for (i = 0; i < time_arr_size; i++)
		arr_ts[i] = -1;
	ts_idx = 0;
	before_called_ts = 0;
	is_first_latency = 1;

	return 0;
}

static long get_lmk_latency(short min_score_adj)
{
	unsigned int timediff_ms;

	if (min_score_adj <= 900) {
		int arr_size = 0;
		if (is_first_latency) {
			before_called_ts = jiffies;
			is_first_latency = 0;
		} else {
			timediff_ms = jiffies_to_msecs(jiffies - before_called_ts);
			before_called_ts = jiffies;
			arr_ts[ts_idx++] = timediff_ms;
			ts_idx %= time_arr_size;
			if (arr_ts[ts_idx] == -1)
				return -1;
			else
				arr_size = time_arr_size;
			return calc_ts_avg(arr_ts, arr_size);
		}
	} else {
		reset_latency();
	}

	return -1;
}

static enum alloc_pressure check_memory_allocation_pressure(short min_score_adj)
{
	long avg_latency = 0;
	if (!current_is_kswapd()) {
		lowmem_print(3, "It's direct reclaim\n");
		return PRESSURE_HIGH;
	}


	avg_latency = get_lmk_latency(min_score_adj);
	if (avg_latency > 0 && avg_latency < avg_treshold) {
		lowmem_print(3, "Check Latency %ldmsec\n", avg_latency);
		reset_latency();
		return PRESSURE_HIGH;
	}

	return PRESSURE_NORMAL;
}
#endif

static int can_use_cma_pages(gfp_t gfp_mask)
{
	int can_use = 0;
	int mtype = gfpflags_to_migratetype(gfp_mask);
	int i = 0;
	int *mtype_fallbacks = get_migratetype_fallbacks(mtype);

	if (is_migrate_cma(mtype)) {
		can_use = 1;
	} else {
		for (i = 0;; i++) {
			int fallbacktype = mtype_fallbacks[i];

			if (is_migrate_cma(fallbacktype)) {
				can_use = 1;
				break;
			}

			if (fallbacktype == MIGRATE_TYPES)
				break;
		}
	}
	return can_use;
}

void tune_lmk_zone_param(struct zonelist *zonelist, int classzone_idx,
					int *other_free, int *other_file,
					int use_cma_pages)
{
	struct zone *zone;
	struct zoneref *zoneref;
	int zone_idx;

	for_each_zone_zonelist(zone, zoneref, zonelist, MAX_NR_ZONES) {
		zone_idx = zonelist_zone_idx(zoneref);
		if (zone_idx == ZONE_MOVABLE) {
			if (!use_cma_pages && other_free)
				*other_free -=
				    zone_page_state(zone, NR_FREE_CMA_PAGES);
			continue;
		}

		if (zone_idx > classzone_idx) {
			if (other_free != NULL)
				*other_free -= zone_page_state(zone,
							       NR_FREE_PAGES);
			if (other_file != NULL)
				*other_file -= zone_page_state(zone,
					NR_ZONE_INACTIVE_FILE) +
					zone_page_state(zone,
					NR_ZONE_ACTIVE_FILE);
		} else if (zone_idx < classzone_idx) {
			if (zone_watermark_ok(zone, 0, 0, classzone_idx, 0) &&
			    other_free) {
				if (!use_cma_pages) {
					*other_free -= min(
					  zone->lowmem_reserve[classzone_idx] +
					  zone_page_state(
					    zone, NR_FREE_CMA_PAGES),
					  zone_page_state(
					    zone, NR_FREE_PAGES));
				} else {
					*other_free -=
					  zone->lowmem_reserve[classzone_idx];
				}
			} else {
				if (other_free)
					*other_free -=
					  zone_page_state(zone, NR_FREE_PAGES);
			}
		}
	}
}

#ifdef CONFIG_HIGHMEM
static void adjust_gfp_mask(gfp_t *gfp_mask)
{
	struct zone *preferred_zone;
	struct zoneref *zref;
	struct zonelist *zonelist;
	enum zone_type high_zoneidx;

	if (current_is_kswapd()) {
		zonelist = node_zonelist(0, *gfp_mask);
		high_zoneidx = gfp_zone(*gfp_mask);
		zref = first_zones_zonelist(zonelist, high_zoneidx, NULL);
		preferred_zone = zref->zone;

		if (high_zoneidx == ZONE_NORMAL) {
			if (zone_watermark_ok_safe(
					preferred_zone, 0,
					high_wmark_pages(preferred_zone), 0))
				*gfp_mask |= __GFP_HIGHMEM;
		} else if (high_zoneidx == ZONE_HIGHMEM) {
			*gfp_mask |= __GFP_HIGHMEM;
		}
	}
}
#else
static void adjust_gfp_mask(gfp_t *unused)
{
}
#endif

void tune_lmk_param(int *other_free, int *other_file, struct shrink_control *sc)
{
	gfp_t gfp_mask;
	struct zone *preferred_zone;
	struct zoneref *zref;
	struct zonelist *zonelist;
	enum zone_type high_zoneidx, classzone_idx;
	unsigned long balance_gap;
	int use_cma_pages;

	gfp_mask = sc->gfp_mask;
	adjust_gfp_mask(&gfp_mask);

	zonelist = node_zonelist(0, gfp_mask);
	high_zoneidx = gfp_zone(gfp_mask);
	zref = first_zones_zonelist(zonelist, high_zoneidx, NULL);
	preferred_zone = zref->zone;
	classzone_idx = zone_idx(preferred_zone);
	use_cma_pages = can_use_cma_pages(gfp_mask);

	balance_gap = min(low_wmark_pages(preferred_zone),
			  (preferred_zone->present_pages +
			   100-1) /
			   100);

	if (likely(current_is_kswapd() && zone_watermark_ok(preferred_zone, 0,
			  high_wmark_pages(preferred_zone) + SWAP_CLUSTER_MAX +
			  balance_gap, 0, 0))) {
		if (lmk_fast_run)
			tune_lmk_zone_param(zonelist, classzone_idx, other_free,
				       other_file, use_cma_pages);
		else
			tune_lmk_zone_param(zonelist, classzone_idx, other_free,
				       NULL, use_cma_pages);

		if (zone_watermark_ok(preferred_zone, 0, 0, _ZONE, 0)) {
			if (!use_cma_pages) {
				*other_free -= min(
				  preferred_zone->lowmem_reserve[_ZONE]
				  + zone_page_state(
				    preferred_zone, NR_FREE_CMA_PAGES),
				  zone_page_state(
				    preferred_zone, NR_FREE_PAGES));
			} else {
				*other_free -=
				  preferred_zone->lowmem_reserve[_ZONE];
			}
		} else {
			*other_free -= zone_page_state(preferred_zone,
						      NR_FREE_PAGES);
		}

		lowmem_print(4, "lowmem_shrink of kswapd tunning for highmem "
			     "ofree %d, %d\n", *other_free, *other_file);
	} else {
		tune_lmk_zone_param(zonelist, classzone_idx, other_free,
			       other_file, use_cma_pages);

		if (!use_cma_pages) {
			*other_free -=
			  zone_page_state(preferred_zone, NR_FREE_CMA_PAGES);
		}

		lowmem_print(4, "lowmem_shrink tunning for others ofree %d, "
			     "%d\n", *other_free, *other_file);
	}
}

static unsigned long lowmem_scan(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	unsigned long rem = 0;
	int tasksize;
	int i;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
	int selected_tasksize = 0;
	short selected_oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES);
	int other_file = global_node_page_state(NR_FILE_PAGES) -
				global_node_page_state(NR_SHMEM) -
				global_node_page_state(NR_UNEVICTABLE) -
				total_swapcache_pages();
	enum zone_type high_zoneidx = gfp_zone(sc->gfp_mask);
	int d_state_is_found = 0;
	short other_min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int to_be_aggressive = 0;
#ifdef CONFIG_HSWAP
	int reclaimed_cnt = 0, reclaimable_cnt = 0, shrink_task_cnt = 0;
	int hswap_tasksize = 0;
	int swapsize = 0, selected_swapsize = 0;
	struct task_struct *hswap_kill_selected = NULL;
	int kill_reason = KILL_LMK;
#endif

#ifdef CONFIG_MIGRATE_HIGHORDER
	other_free -= global_page_state(NR_FREE_HIGHORDER_PAGES);
#endif

	if (!spin_trylock(&lowmem_shrink_lock)) {
		lowmem_print(4, "lowmem_shrink lock failed\n");
		return SHRINK_STOP;
	}

#ifdef CONFIG_HSWAP
	if (!mutex_trylock(&reclaim_mutex)) {
		spin_unlock(&lowmem_shrink_lock);
		return 0;
	}
	mutex_unlock(&reclaim_mutex);
#endif

	/*
	 * Check whether it is caused by low memory in lower zone(s)!
	 * This will help solve over-reclaiming situation while total number
	 * of free pages is enough, but lower one(s) is(are) under low memory.
	 */
	if (lowmem_check_status_by_zone(high_zoneidx, &other_free, &other_file)
			> 0)
		other_min_score_adj = 0;

	other_min_score_adj =
		min(other_min_score_adj,
		    lowmem_amr_check(&to_be_aggressive, other_file));

	/* Let other_free be positive or zero */
	if (other_free < 0)
		other_free = 0;

#ifndef CONFIG_HSWAP
	tune_lmk_param(&other_free, &other_file, sc);
#endif

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
			if (to_be_aggressive != 0 && i > 3) {
				i -= to_be_aggressive;
				if (i < 3)
					i = 3;
			}
			min_score_adj = lowmem_adj[i];
			break;
		}
	}

	/* Compute suitable min_score_adj */
	min_score_adj = min(min_score_adj, other_min_score_adj);

	lowmem_print(4, "lowmem_scan %lu, %x, ofree %d %d, ma %hd\n",
		     sc->nr_to_scan, sc->gfp_mask, other_free,
		     other_file, min_score_adj);

	if (min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_scan %lu, %x, return 0\n",
			     sc->nr_to_scan, sc->gfp_mask);
		spin_unlock(&lowmem_shrink_lock);
		return 0;
	}

	selected_oom_score_adj = min_score_adj;

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		if (time_before_eq(jiffies, lowmem_deathpending_timeout)) {
			if (task_lmk_waiting(tsk)) {
#ifdef CONFIG_HSWAP
				goto end_lmk;
#else
				rcu_read_unlock();
				spin_unlock(&lowmem_shrink_lock);
				return 0;
#endif
			}
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		/* Bypass D-state process */
		if (p->state & TASK_UNINTERRUPTIBLE) {
			lowmem_print(2,
				     "lowmem_scan filter D state process: %d (%s) state:0x%lx\n",
				     p->pid, p->comm, p->state);
			task_unlock(p);
			d_state_is_found = 1;
			continue;
		}

		oom_score_adj = p->signal->oom_score_adj;
#ifdef CONFIG_HSWAP
		if (p->signal->reclaimed)
			reclaimed_cnt++;

		if (oom_score_adj >= OOM_SCORE_SERVICE_B_ADJ) {
			if (shrink_task_cnt < SHRINK_TASK_MAX_CNT)
				shrink_task[shrink_task_cnt++] = p;
			if (!p->signal->reclaimed)
				reclaimable_cnt++;
		}
#endif
		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}

		tasksize = get_mm_rss(p->mm) +
			get_mm_counter(p->mm, MM_SWAPENTS);

#ifdef CONFIG_HSWAP
		swapsize = get_mm_counter(p->mm, MM_SWAPENTS);
#endif
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
#ifdef CONFIG_HSWAP
		selected_swapsize = swapsize;
#endif
		lowmem_print(2, "select '%s' (%d), adj %hd, size %d, to kill\n",
			     p->comm, p->pid, oom_score_adj, tasksize);
	}
	if (selected) {
		long cache_size = other_file * (long)(PAGE_SIZE / 1024);
		long cache_limit = minfree * (long)(PAGE_SIZE / 1024);
		long free = other_free * (long)(PAGE_SIZE / 1024);

#ifdef CONFIG_HSWAP
		if (min_score_adj < OOM_SCORE_SERVICE_B_ADJ) {
			selected_tasksize += selected_swapsize;
			goto hswap_kill;
		}

		if (check_memory_allocation_pressure(min_score_adj) == PRESSURE_HIGH) {
			kill_reason = KILL_MEMORY_PRESSURE;
			lowmem_print(3, "Memory Alloctions is High\n");
			goto hswap_kill;
		}

		if (!reclaimable_cnt &&
				(min_score_adj > OOM_SCORE_CACHED_APP_MIN_ADJ)) {
			rem = SHRINK_STOP;
			goto end_lmk;
		}

		if (reclaimable_cnt && selected_task == NULL && mutex_trylock(&reclaim_mutex)) {
			selected_task = find_suitable_reclaim(shrink_task_cnt, &hswap_tasksize);
			if (selected_task) {
				unsigned long flags;

				if (lock_task_sighand(selected_task, &flags)) {
					selected_task->signal->reclaimed = 1;
					unlock_task_sighand(selected_task, &flags);
				}
				get_task_struct(selected_task);
				complete(&reclaim_completion);
				rem += hswap_tasksize;
				lowmem_print(1, "Reclaiming '%s' (%d), adj %hd,\n" \
						"   top time = %ld, top count %d,\n" \
						"   to free %ldkB on behalf of '%s' (%d) because\n" \
						"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n" \
						"   Free memory is %ldkB above reserved.\n",
						selected_task->comm, selected_task->pid,
						selected_task->signal->oom_score_adj,
						selected_task->signal->top_time,
						selected_task->signal->top_count,
						hswap_tasksize * (long)(PAGE_SIZE / 1024),
						current->comm, current->pid,
						other_file * (long)(PAGE_SIZE / 1024),
						minfree * (long)(PAGE_SIZE / 1024),
						min_score_adj,
						other_free * (long)(PAGE_SIZE / 1024));
				lowmem_print(3, "reclaimed cnt = %d, reclaimable cont = %d, min oom score= %hd\n",
						reclaimed_cnt, reclaimable_cnt, min_score_adj);
				mutex_unlock(&reclaim_mutex);
				goto end_lmk;
			} else {
				mutex_unlock(&reclaim_mutex);
				kill_reason = KILL_SWAP_FULL;
			}
		} else {
			if (!reclaimable_cnt)
				kill_reason = KILL_NO_RECLAIMABLE;
			else
				kill_reason = KILL_RECLAIMING;
		}

hswap_kill:
		if (shrink_task_cnt > 0) {
			hswap_kill_selected = find_suitable_kill_task(shrink_task_cnt, &selected_tasksize);
			if (hswap_kill_selected)
				selected = hswap_kill_selected;
		}
#endif
		task_lock(selected);
		send_sig(SIGKILL, selected, 0);
		if (selected->mm)
			task_set_lmk_waiting(selected);
		task_unlock(selected);
		trace_lowmemory_kill(selected, cache_size, cache_limit, free);
		++lmk_kill_cnt;
#ifndef CONFIG_HSWAP
		lowmem_print(1, "Killing '%s' (%d) (tgid %d), adj %hd,\n"
#else
		lowmem_print(1, "%s '%s' (%d) (tgid %d), adj %hd,\n"
				 " reclaim_cnt %d, top (%ld, %d), reason %s\n"
#endif
				 "   to free %ldkB on behalf of '%s' (%d) because\n"
				 "   cache %ldkB is below limit %ldkB for oom_score_adj %hd (%hd)\n"
				 "   Free memory is %ldkB above reserved(decrease %d level)\n",
#ifdef CONFIG_HSWAP
				 hswap_kill_selected ? "HSWAP Killing" : "Orig Killing",
#endif
			     selected->comm, selected->pid, selected->tgid,
				 selected->signal->oom_score_adj,
#ifdef CONFIG_HSWAP
				 reclaimable_cnt,
				 selected->signal->top_time,
				 selected->signal->top_count,
				 kill_reason_str[kill_reason],
#endif
				 selected_tasksize * (long)(PAGE_SIZE / 1024),
				 current->comm, current->pid,
				 cache_size, cache_limit,
				 min_score_adj, other_min_score_adj,
				 free, to_be_aggressive);
		lowmem_deathpending_timeout = jiffies + HZ;
		lowmem_trigger_warning(selected, selected_oom_score_adj);
		rem += selected_tasksize;
#ifdef CONFIG_HSWAP
		if (kill_reason == KILL_MEMORY_PRESSURE)
			rem = SHRINK_STOP;

		lowmem_print(3, "reclaimed cnt = %d, reclaim cont = %d, min oom score= %hd\n",
				reclaimed_cnt, reclaimable_cnt, min_score_adj);
#endif
	} else {
		if (d_state_is_found == 1)
			lowmem_print(2,
				     "No selected (full of D-state processes at %d)\n",
				     (int)min_score_adj);
	}

	lowmem_print(4, "lowmem_scan %lu, %x, return %lu\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
#ifdef CONFIG_HSWAP
end_lmk:
	reclaim_arr_free(shrink_task_cnt);
#endif
	rcu_read_unlock();
	spin_unlock(&lowmem_shrink_lock);

	/* dump more memory info outside the lock */
	if (selected && selected_oom_score_adj <= lowmem_no_warn_adj &&
	    min_score_adj <= lowmem_warn_adj)
		dump_memory_status(selected_oom_score_adj);

#ifdef MTK_LMK_USER_EVENT
	/* Send uevent if needed */
	if (mtklmk_initialized && current_is_kswapd() && mtklmk_uevent_timeout)
		mtklmk_uevent(min_score_adj, minfree);
#endif

	return rem;
}

static struct shrinker lowmem_shrinker = {
	.scan_objects = lowmem_scan,
	.count_objects = lowmem_count,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
#ifdef CONFIG_HSWAP
	struct task_struct *reclaim_tsk;
	struct task_struct *reset_top_time_tsk;
	int i = 0;
#endif

	if (IS_ENABLED(CONFIG_ZRAM) &&
	    IS_ENABLED(CONFIG_MTK_GMO_RAM_OPTIMIZE))
		vm_swappiness = 100;

	register_shrinker(&lowmem_shrinker);

#ifdef MTK_LMK_USER_EVENT
	/* initialize work for uevent */
	INIT_WORK(&mtklmk_work, mtklmk_async_uevent);

	/* register as misc device */
	if (!misc_register(&mtklmk_misc)) {
		pr_info("%s: successful to register misc device!\n", __func__);
		mtklmk_initialized = 1;
	}
#endif

#ifdef CONFIG_HSWAP
	reclaim_tsk = kthread_run(reclaim_task_thread, NULL, "reclaim_task");
	reset_top_time_tsk = kthread_run(reset_task_time_thread, NULL, "reset_task");

	for (; i < TIME_ARR_SIZE; i++)
		arr_ts[i] = -1;
#endif

	return 0;
}
device_initcall(lowmem_init);

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

/*
 * not really modular, but the easiest way to keep compat with existing
 * bootargs behaviour is to continue using module_param here.
 */
module_param_named(cost, lowmem_shrinker.seeks, int, 0644);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
module_param_cb(adj, &lowmem_adj_array_ops,
		.arr = &__param_arr_adj,
		0644);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size, 0644);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size, 0644);
module_param_named(debug_level, lowmem_debug_level, uint, 0644);
module_param_named(debug_adj, lowmem_warn_adj, short, 0644);
module_param_named(no_debug_adj, lowmem_no_warn_adj, short, 0644);
module_param_named(lmk_kill_cnt, lmk_kill_cnt, int, S_IRUGO);
#ifdef CONFIG_HSWAP
module_param_named(lmk_reclaim_cnt, lmk_reclaim_cnt, int, S_IRUGO);
#endif
module_param_named(lmk_fast_run, lmk_fast_run, int, S_IRUGO | S_IWUSR);

