/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_nrf_hfxo64m

// #include <zephyr/drivers/clock_control/clock_control_nrf2_common.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_hfxo64m.h>
LOG_MODULE_REGISTER(clock_control_hfxo64m, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "multiple instances not supported");

struct dev_data_hfxo64m {
	struct onoff_manager mgr;
	onoff_notify_fn notify;
	struct k_timer timer;
	sys_snode_t hfxo64m_node;
#if defined(CONFIG_ZERO_LATENCY_IRQS)
	uint16_t request_count;
#endif /* CONFIG_ZERO_LATENCY_IRQS */
	k_timeout_t start_up_time;
};

struct dev_config_hfxo64m {
	uint32_t fixed_frequency;
	uint16_t fixed_accuracy;
};

int api_nosys_on_off(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	return -ENOSYS;
}


#if defined(CONFIG_ZERO_LATENCY_IRQS)
static uint32_t full_irq_lock(void)
{
	uint32_t mcu_critical_state;

	mcu_critical_state = __get_PRIMASK();
	__disable_irq();

	return mcu_critical_state;
}

static void full_irq_unlock(uint32_t mcu_critical_state)
{
	__set_PRIMASK(mcu_critical_state);
}
#endif /* CONFIG_ZERO_LATENCY_IRQS */

static void hfxo64m_start_up_timer_handler(struct k_timer *timer)
{
	struct dev_data_hfxo64m *dev_data =
		CONTAINER_OF(timer, struct dev_data_hfxo64m, timer);

	/* from nrf54h20 hfxo: 
	 * "In specific cases, the hfxoSTARTED event might not be set even
	 * though the hfxo64m has started (this is a hardware issue that will
	 * be fixed). For now, the hfxo is simply assumed to be started
	 * after its configured start-up time expires."
	 */

	LOG_DBG("HFXOSTARTED: %u",
		nrf_hfxo64m_event_check(NRF_HFXO64M, NRF_HFXO64M_EVENT_STARTED));

	if (dev_data->notify) {
		dev_data->notify(&dev_data->mgr, 0);
	}
}

static void start_hfxo64m(struct dev_data_hfxo64m *dev_data)
{
	LOG_DBG("Starting HFXO");
	NRF_CLOCK->TASKS_XOSTART = 1;
	// while (!NRF_CLOCK->EVENTS_XOSTARTED) {}

	// nrf_lrcconf_event_clear(NRF_LRCCONF010, NRF_LRCCONF_EVENT_hfxo64mSTARTED);
	// soc_lrcconf_poweron_request(&dev_data->hfxo64m_node, NRF_LRCCONF_POWER_MAIN);
	// nrf_lrcconf_task_trigger(NRF_LRCCONF010, NRF_LRCCONF_TASK_REQhfxo64m);
}

static void request_hfxo64m(struct dev_data_hfxo64m *dev_data)
{
#if defined(CONFIG_ZERO_LATENCY_IRQS)
	unsigned int key;

	key = full_irq_lock();
	if (dev_data->request_count == 0) {
		start_hfxo64m(dev_data);
	}

	dev_data->request_count++;
	full_irq_unlock(key);
#else
	start_hfxo64m(dev_data);
#endif /* CONFIG_ZERO_LATENCY_IRQS */
}

#if IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS)
void nrf_clock_control_hfxo64m_request(void)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	struct dev_data_hfxo64m *dev_data = dev->data;

	request_hfxo64m(dev_data);
}
#endif /* CONFIG_ZERO_LATENCY_IRQS */

static void onoff_start_hfxo64m(struct onoff_manager *mgr, onoff_notify_fn notify)
{
	struct dev_data_hfxo64m *dev_data =
		CONTAINER_OF(mgr, struct dev_data_hfxo64m, mgr);

	dev_data->notify = notify;
	request_hfxo64m(dev_data);

	/* Due to a hardware issue, the hfxo64mSTARTED event is currently
	 * unreliable. Hence the timer is used to simply wait the expected
	 * start-up time. To be removed once the hardware is fixed.
	 */
	k_timer_start(&dev_data->timer, dev_data->start_up_time, K_NO_WAIT);
}

static void stop_hfxo64m(struct dev_data_hfxo64m *dev_data)
{
	LOG_DBG("Stopping HFXO");
	NRF_CLOCK->TASKS_XOSTOP = 1;
	// nrf_lrcconf_task_trigger(NRF_LRCCONF010, NRF_LRCCONF_TASK_STOPREQhfxo64m);
	// soc_lrcconf_poweron_release(&dev_data->hfxo64m_node, NRF_LRCCONF_POWER_MAIN);
}

static void release_hfxo64m(struct dev_data_hfxo64m *dev_data)
{
#if IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS)
	unsigned int key;

	key = full_irq_lock();
	if (dev_data->request_count < 1) {
		full_irq_unlock(key);
		/* Misuse of the API, release without request? */
		__ASSERT_NO_MSG(false);
		/* In case asserts are disabled early return due to no requests pending */
		return;
	}

	dev_data->request_count--;
	if (dev_data->request_count < 1) {
		stop_hfxo64m(dev_data);
	}

	full_irq_unlock(key);
#else
	stop_hfxo64m(dev_data);
#endif /* CONFIG_ZERO_LATENCY_IRQS */
}

#if IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS)
void nrf_clock_control_hfxo64m_release(void)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	struct dev_data_hfxo64m *dev_data = dev->data;

	release_hfxo64m(dev_data);
}
#endif /* IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) */

static void onoff_stop_hfxo64m(struct onoff_manager *mgr, onoff_notify_fn notify)
{
	struct dev_data_hfxo64m *dev_data =
		CONTAINER_OF(mgr, struct dev_data_hfxo64m, mgr);

	release_hfxo64m(dev_data);
	notify(mgr, 0);
}

static bool is_clock_spec_valid(const struct device *dev,
				const struct nrf_clock_spec *spec)
{
	const struct dev_config_hfxo64m *dev_config = dev->config;

	if (spec->frequency > dev_config->fixed_frequency) {
		LOG_ERR("invalid frequency");
		return false;
	}

	/* Signal an error if an accuracy better than available is requested. */
	if (spec->accuracy &&
	    spec->accuracy != NRF_CLOCK_CONTROL_ACCURACY_MAX &&
	    spec->accuracy < dev_config->fixed_accuracy) {
		LOG_ERR("invalid accuracy");
		return false;
	}

	/* Consider hfxo64m precision high, skip checking what is requested. */

	return true;
}

static int api_request_hfxo64m(const struct device *dev,
			    const struct nrf_clock_spec *spec,
			    struct onoff_client *cli)
{
	struct dev_data_hfxo64m *dev_data = dev->data;

	if (spec && !is_clock_spec_valid(dev, spec)) {
		return -EINVAL;
	}

	return onoff_request(&dev_data->mgr, cli);
}

static int api_release_hfxo64m(const struct device *dev,
			    const struct nrf_clock_spec *spec)
{
	struct dev_data_hfxo64m *dev_data = dev->data;

	if (spec && !is_clock_spec_valid(dev, spec)) {
		return -EINVAL;
	}

	return onoff_release(&dev_data->mgr);
}

static int api_cancel_or_release_hfxo64m(const struct device *dev,
				      const struct nrf_clock_spec *spec,
				      struct onoff_client *cli)
{
	struct dev_data_hfxo64m *dev_data = dev->data;

	if (spec && !is_clock_spec_valid(dev, spec)) {
		return -EINVAL;
	}

	return onoff_cancel_or_release(&dev_data->mgr, cli);
}

static int api_get_rate_hfxo64m(const struct device *dev,
			     clock_control_subsys_t sys,
			     uint32_t *rate)
{
	ARG_UNUSED(sys);

	const struct dev_config_hfxo64m *dev_config = dev->config;

	*rate = dev_config->fixed_frequency;

	return 0;
}

static int init_hfxo64m(const struct device *dev)
{
	LOG_DBG("Initialising HFXO");
	struct dev_data_hfxo64m *dev_data = dev->data;
	static const struct onoff_transitions transitions = {
		.start = onoff_start_hfxo64m,
		.stop = onoff_stop_hfxo64m
	};
	uint32_t start_up_time;
	int rc;

	rc = onoff_manager_init(&dev_data->mgr, &transitions);
	if (rc < 0) {
		return rc;
	}


	dev_data->start_up_time = K_USEC(50);

	k_timer_init(&dev_data->timer, hfxo64m_start_up_timer_handler, NULL);

	return 0;
}

static DEVICE_API(nrf_clock_control, drv_api_hfxo64m) = {
	.std_api = {
		.on = api_nosys_on_off,
		.off = api_nosys_on_off,
		.get_rate = api_get_rate_hfxo64m,
	},
	.request = api_request_hfxo64m,
	.release = api_release_hfxo64m,
	.cancel_or_release = api_cancel_or_release_hfxo64m,
};

static struct dev_data_hfxo64m data_hfxo64m;

static const struct dev_config_hfxo64m config_hfxo64m = {
	.fixed_frequency = DT_INST_PROP(0, clock_frequency),
	.fixed_accuracy = DT_INST_PROP(0, accuracy_ppm),
};

// DEVICE_DT_INST_DEFINE(0, init_hfxo64m, NULL,
// 		      &data_hfxo64m, &config_hfxo64m,
// 		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
// 		      &drv_api_hfxo64m);

DEVICE_DT_INST_DEFINE(0, init_hfxo64m, NULL,
	&data_hfxo64m, &config_hfxo64m,
	PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
	NULL);
