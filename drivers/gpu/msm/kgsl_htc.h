#ifndef KGSL_HTC_H
#define KGSL_HTC_H

#include <linux/msm_kgsl.h>

struct kgsl_device;
struct idr;

struct kgsl_driver_htc_priv {
	struct work_struct work;
	unsigned long next_jiffies;
};

/* init data structures which plug-in kgsl_driver structure
 */
int kgsl_driver_htc_init(struct kgsl_driver_htc_priv *priv);

/* init htc feature in kgsl_device_platform_probe function
 */
int kgsl_device_htc_init(struct kgsl_device *device);

/* Dump pid informations of all contexts *
 * caller need to hold context_lock      */
void kgsl_dump_contextpid_locked(struct idr *context_idr);

/* enter panic/kill process when GPU fault happened */
void adreno_fault_panic(struct kgsl_device *device, unsigned int pid, int fault);

#endif
