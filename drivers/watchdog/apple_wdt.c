// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC Watchdog driver
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/crash_dump.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/watchdog.h>

/*
 * Apple Watchdog MMIO registers
 *
 * This HW block has three separate watchdogs. WD0 resets the machine
 * to recovery mode and is not very useful for us. WD1 and WD2 trigger a normal
 * machine reset. WD0 additionally supports a configurable interrupt.
 * This information can be used to implement pretimeout support at a later time.
 *
 * APPLE_WDT_WDx_CUR_TIME is a simple counter incremented for each tick of the
 * reference clock. It can also be overwritten to any value.
 * Whenever APPLE_WDT_CTRL_RESET_EN is set in APPLE_WDT_WDx_CTRL and
 * APPLE_WDT_WDx_CUR_TIME >= APPLE_WDT_WDx_BITE_TIME the entire machine is
 * reset.
 * Whenever APPLE_WDT_CTRL_IRQ_EN is set and APPLE_WDTx_WD1_CUR_TIME >=
 * APPLE_WDTx_WD1_BARK_TIME an interrupt is triggered and
 * APPLE_WDT_CTRL_IRQ_STATUS is set. The interrupt can be cleared by writing
 * 1 to APPLE_WDT_CTRL_IRQ_STATUS.
 */
#define APPLE_WDT_WD0_CUR_TIME		0x00
#define APPLE_WDT_WD0_BITE_TIME		0x04
#define APPLE_WDT_WD0_BARK_TIME		0x08
#define APPLE_WDT_WD0_CTRL		0x0c

#define APPLE_WDT_WD1_CUR_TIME		0x10
#define APPLE_WDT_WD1_BITE_TIME		0x14
#define APPLE_WDT_WD1_CTRL		0x1c

#define APPLE_WDT_WD2_CUR_TIME		0x20
#define APPLE_WDT_WD2_BITE_TIME		0x24
#define APPLE_WDT_WD2_CTRL		0x2c

#define APPLE_WDT_CTRL_IRQ_EN		BIT(0)
#define APPLE_WDT_CTRL_IRQ_STATUS	BIT(1)
#define APPLE_WDT_CTRL_RESET_EN		BIT(2)

#define APPLE_WDT_TIMEOUT_DEFAULT	30

/*
 * Suspend-to-idle wake timer
 *
 * A suspend to idle is left only for a wakeup event. Where the RTC alarm has
 * no interrupt (T6050: the PMU's alarm is behind a generation 4 SPMI
 * controller, whose interrupts are not supported), nothing but a person can
 * end one. WD0's bark interrupt is a timer that can: with s2idle_wake_s set,
 * the suspend arms WD0 to interrupt (never to reset) that many seconds later,
 * and the interrupt is a wakeup event. Without RESET_EN, WD0 does nothing
 * else.
 *
 * As a last resort, s2idle_fallback_s seconds after that wake was due, WD1
 * resets the machine: a wake that does not come cannot leave it asleep. The
 * watchdog is otherwise stopped across a suspend, as before. Both are undone
 * by the resume. WD0 and WD1 count the 24 MHz reference, so the sum is at
 * most about 178 s.
 */
static unsigned int s2idle_wake_s;
module_param(s2idle_wake_s, uint, 0644);
MODULE_PARM_DESC(s2idle_wake_s, "Wake from suspend-to-idle after this many seconds (0: off)");

static unsigned int s2idle_fallback_s = 60;
module_param(s2idle_fallback_s, uint, 0644);
MODULE_PARM_DESC(s2idle_fallback_s,
		 "With s2idle_wake_s, reset this many seconds after a wake that did not come (0: off)");

struct apple_wdt {
	struct watchdog_device wdd;
	void __iomem *regs;
	unsigned long clk_rate;
	/* WD0's bark interrupt, or <= 0 without one */
	int irq;
	bool wake_armed;
	bool fallback_armed;
	bool barked;
};

static struct apple_wdt *to_apple_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct apple_wdt, wdd);
}

static int apple_wdt_start(struct watchdog_device *wdd)
{
	struct apple_wdt *wdt = to_apple_wdt(wdd);

	writel_relaxed(0, wdt->regs + APPLE_WDT_WD1_CUR_TIME);
	writel_relaxed(APPLE_WDT_CTRL_RESET_EN, wdt->regs + APPLE_WDT_WD1_CTRL);

	return 0;
}

static int apple_wdt_stop(struct watchdog_device *wdd)
{
	struct apple_wdt *wdt = to_apple_wdt(wdd);

	writel_relaxed(0, wdt->regs + APPLE_WDT_WD1_CTRL);

	return 0;
}

static int apple_wdt_ping(struct watchdog_device *wdd)
{
	struct apple_wdt *wdt = to_apple_wdt(wdd);

	writel_relaxed(0, wdt->regs + APPLE_WDT_WD1_CUR_TIME);

	return 0;
}

static int apple_wdt_set_timeout(struct watchdog_device *wdd, unsigned int s)
{
	struct apple_wdt *wdt = to_apple_wdt(wdd);
	u32 actual;

	writel_relaxed(0, wdt->regs + APPLE_WDT_WD1_CUR_TIME);

	actual = min(s, wdd->max_hw_heartbeat_ms / 1000);
	writel_relaxed(wdt->clk_rate * actual, wdt->regs + APPLE_WDT_WD1_BITE_TIME);

	wdd->timeout = s;

	return 0;
}

static unsigned int apple_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct apple_wdt *wdt = to_apple_wdt(wdd);
	u32 cur_time, reset_time;

	cur_time = readl_relaxed(wdt->regs + APPLE_WDT_WD1_CUR_TIME);
	reset_time = readl_relaxed(wdt->regs + APPLE_WDT_WD1_BITE_TIME);

	return (reset_time - cur_time) / wdt->clk_rate;
}

static int apple_wdt_restart(struct watchdog_device *wdd, unsigned long mode,
			     void *cmd)
{
	struct apple_wdt *wdt = to_apple_wdt(wdd);

	writel_relaxed(APPLE_WDT_CTRL_RESET_EN, wdt->regs + APPLE_WDT_WD1_CTRL);
	writel_relaxed(0, wdt->regs + APPLE_WDT_WD1_BITE_TIME);
	writel_relaxed(0, wdt->regs + APPLE_WDT_WD1_CUR_TIME);

	/*
	 * Flush writes and then wait for the SoC to reset. Even though the
	 * reset is queued almost immediately experiments have shown that it
	 * can take up to ~120-125ms until the SoC is actually reset. Just
	 * wait 150ms here to be safe.
	 */
	(void)readl(wdt->regs + APPLE_WDT_WD1_CUR_TIME);
	mdelay(150);

	return 0;
}

static const struct watchdog_ops apple_wdt_ops = {
	.owner = THIS_MODULE,
	.start = apple_wdt_start,
	.stop = apple_wdt_stop,
	.ping = apple_wdt_ping,
	.set_timeout = apple_wdt_set_timeout,
	.get_timeleft = apple_wdt_get_timeleft,
	.restart = apple_wdt_restart,
};

static const struct watchdog_info apple_wdt_info = {
	.identity = "Apple SoC Watchdog",
	.options = WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT,
};

/* WD0's bark: the suspend-to-idle wake timer has run out. */
static irqreturn_t apple_wdt_bark(int irq, void *data)
{
	struct device *dev = data;
	struct apple_wdt *wdt = dev_get_drvdata(dev);
	u32 ctrl = readl_relaxed(wdt->regs + APPLE_WDT_WD0_CTRL);

	if (!(ctrl & APPLE_WDT_CTRL_IRQ_STATUS))
		return IRQ_NONE;
	/* Acknowledge and disarm: one wake per suspend. */
	writel_relaxed(APPLE_WDT_CTRL_IRQ_STATUS, wdt->regs + APPLE_WDT_WD0_CTRL);
	wdt->barked = true;
	pm_wakeup_dev_event(dev, 0, true);
	return IRQ_HANDLED;
}

static int apple_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_wdt *wdt;
	struct clk *clk;
	u32 wdt_ctrl;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->regs))
		return PTR_ERR(wdt->regs);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	wdt->clk_rate = clk_get_rate(clk);
	if (!wdt->clk_rate)
		return -EINVAL;

	platform_set_drvdata(pdev, wdt);

	wdt->wdd.parent = dev;
	wdt->wdd.ops = &apple_wdt_ops;
	wdt->wdd.info = &apple_wdt_info;
	wdt->wdd.max_hw_heartbeat_ms = U32_MAX / wdt->clk_rate * 1000;
	wdt->wdd.timeout = APPLE_WDT_TIMEOUT_DEFAULT;

	wdt_ctrl = readl_relaxed(wdt->regs + APPLE_WDT_WD1_CTRL);
	if ((wdt_ctrl & APPLE_WDT_CTRL_RESET_EN) && !is_kdump_kernel())
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);

	watchdog_set_nowayout(&wdt->wdd, WATCHDOG_NOWAYOUT);
	watchdog_init_timeout(&wdt->wdd, 0, dev);
	apple_wdt_set_timeout(&wdt->wdd, wdt->wdd.timeout);

	/*
	 * A crash kernel only saves the dump and restarts. If it stalls, the
	 * machine must still restart rather than hang: arm the watchdog at its
	 * longest timeout and leave it unpinged, unless userspace opens it.
	 */
	if (is_kdump_kernel()) {
		apple_wdt_set_timeout(&wdt->wdd,
				      wdt->wdd.max_hw_heartbeat_ms / 1000);
		apple_wdt_start(&wdt->wdd);
		dev_info(dev, "crash kernel: restarting in %u s unless done\n",
			 wdt->wdd.timeout);
	}
	watchdog_stop_on_unregister(&wdt->wdd);
	watchdog_set_restart_priority(&wdt->wdd, 128);

	/*
	 * The suspend-to-idle wake timer. Its interrupt is enabled only while
	 * the timer is armed, and is left enabled through the suspend so that
	 * its handler reports the wakeup from the idle loop.
	 */
	wdt->irq = platform_get_irq_optional(pdev, 0);
	if (wdt->irq > 0) {
		int ret = devm_request_irq(dev, wdt->irq, apple_wdt_bark,
					   IRQF_NO_AUTOEN | IRQF_NO_SUSPEND,
					   "apple-wdt-s2idle-wake", dev);

		if (!ret)
			ret = devm_device_init_wakeup(dev);
		if (ret) {
			dev_warn(dev, "no suspend-to-idle wake timer: %d\n", ret);
			wdt->irq = 0;
		}
	}

	return devm_watchdog_register_device(dev, &wdt->wdd);
}

static int apple_wdt_resume(struct device *dev)
{
	struct apple_wdt *wdt = dev_get_drvdata(dev);

	if (wdt->wake_armed) {
		disable_irq(wdt->irq);
		writel_relaxed(APPLE_WDT_CTRL_IRQ_STATUS, wdt->regs + APPLE_WDT_WD0_CTRL);
		wdt->wake_armed = false;
		dev_info(dev, "s2idle: %s\n", wdt->barked ? "woken by the wake timer" :
			 "woken before the wake timer");
	}
	if (wdt->fallback_armed) {
		wdt->fallback_armed = false;
		/* Back to the watchdog's own timeout, stopped unless it runs. */
		apple_wdt_stop(&wdt->wdd);
		apple_wdt_set_timeout(&wdt->wdd, wdt->wdd.timeout);
	}

	if (watchdog_active(&wdt->wdd) || watchdog_hw_running(&wdt->wdd))
		apple_wdt_start(&wdt->wdd);

	return 0;
}

/* Arms the wake timer and its fallback, see "Suspend-to-idle wake timer". */
static void apple_wdt_arm_s2idle_wake(struct device *dev, struct apple_wdt *wdt)
{
	unsigned int max_s = wdt->wdd.max_hw_heartbeat_ms / 1000;
	unsigned int wake = min(READ_ONCE(s2idle_wake_s), max_s);
	unsigned int fallback = READ_ONCE(s2idle_fallback_s);

	writel_relaxed(APPLE_WDT_CTRL_IRQ_STATUS, wdt->regs + APPLE_WDT_WD0_CTRL);
	writel_relaxed(0, wdt->regs + APPLE_WDT_WD0_CUR_TIME);
	writel_relaxed(wdt->clk_rate * wake, wdt->regs + APPLE_WDT_WD0_BARK_TIME);
	wdt->barked = false;
	wdt->wake_armed = true;
	enable_irq(wdt->irq);
	writel_relaxed(APPLE_WDT_CTRL_IRQ_EN, wdt->regs + APPLE_WDT_WD0_CTRL);

	if (fallback) {
		fallback = min(wake + fallback, max_s);
		writel_relaxed(0, wdt->regs + APPLE_WDT_WD1_CTRL);
		writel_relaxed(0, wdt->regs + APPLE_WDT_WD1_CUR_TIME);
		writel_relaxed(wdt->clk_rate * fallback, wdt->regs + APPLE_WDT_WD1_BITE_TIME);
		writel_relaxed(APPLE_WDT_CTRL_RESET_EN, wdt->regs + APPLE_WDT_WD1_CTRL);
		wdt->fallback_armed = true;
		dev_info(dev, "s2idle: wake timer %u s, reset at %u s if no wake\n", wake, fallback);
	} else {
		dev_info(dev, "s2idle: wake timer %u s, no fallback\n", wake);
	}
}

static int apple_wdt_suspend(struct device *dev)
{
	struct apple_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd) || watchdog_hw_running(&wdt->wdd))
		apple_wdt_stop(&wdt->wdd);

	if (wdt->irq > 0 && READ_ONCE(s2idle_wake_s) &&
	    pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		apple_wdt_arm_s2idle_wake(dev, wdt);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_wdt_pm_ops, apple_wdt_suspend, apple_wdt_resume);

static const struct of_device_id apple_wdt_of_match[] = {
	{ .compatible = "apple,t8103-wdt" },
	{ .compatible = "apple,wdt" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_wdt_of_match);

static struct platform_driver apple_wdt_driver = {
	.driver = {
		.name = "apple-watchdog",
		.of_match_table = apple_wdt_of_match,
		.pm = pm_sleep_ptr(&apple_wdt_pm_ops),
	},
	.probe = apple_wdt_probe,
};
module_platform_driver(apple_wdt_driver);

MODULE_DESCRIPTION("Apple SoC watchdog driver");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_LICENSE("Dual MIT/GPL");
