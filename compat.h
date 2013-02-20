#ifndef module_driver
/**
 * module_driver() - Helper macro for drivers that don't do anything
 * special in module init/exit. This eliminates a lot of boilerplate.
 * Each module may only use this macro once, and calling it replaces
 * module_init() and module_exit().
 *
 * @__driver: driver name
 * @__register: register function for this driver type
 * @__unregister: unregister function for this driver type
 * @...: Additional arguments to be passed to __register and __unregister.
 *
 * Use this macro to construct bus specific macros for registering
 * drivers, and do not use it on its own.
 */
#define module_driver(__driver, __register, __unregister, ...) \
static int __init __driver##_init(void) \
{ \
	return __register(&(__driver) , ##__VA_ARGS__); \
} \
module_init(__driver##_init); \
static void __exit __driver##_exit(void) \
{ \
	__unregister(&(__driver) , ##__VA_ARGS__); \
} \
module_exit(__driver##_exit);
#endif

#ifndef module_pci_driver
/**
 * module_pci_driver() - Helper macro for registering a PCI driver
 * @__pci_driver: pci_driver struct
 *
 * Helper macro for PCI drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 */
#define module_pci_driver(__pci_driver) \
	module_driver(__pci_driver, pci_register_driver, \
		       pci_unregister_driver)
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
#define videobuf_queue_to_vaddr videobuf_queue_to_vmalloc
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,28)
#define v4l2_file_operations file_operations
#endif


#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,29) && defined(__SOUND_CORE_H)
static inline
int snd_card_create(int idx, const char *id,
		    struct module *module, int extra_size,
		    struct snd_card **card_ret)
{
	struct snd_card *card;

	if (BUG_ON(!card_ret))
		return -EINVAL;

	card = snd_card_new(idx, id, module, extra_size);
	if (card == NULL)
		return -ENOMEM;

	*card_ret = card;
	return 0;
}
#endif
