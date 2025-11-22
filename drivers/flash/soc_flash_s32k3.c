#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <errno.h>
#include <soc.h>

/* Include NXP S32K3 C40 HAL */
// #include "C40_Ip.h"  /* From NXP RTD or HAL */

#define DT_DRV_COMPAT nxp_s32k3_c40

struct flash_s32k3_config {
    uint32_t base_addr;
    size_t flash_size;
    size_t write_block_size;
    size_t erase_block_size;
};

struct flash_s32k3_data {
    struct k_sem sem;
    // C40_Ip_ConfigType flash_config;
};

/* Flash operations must run from RAM on S32K3 */
static int __ramfunc flash_s32k3_erase(const struct device *dev,
                                      off_t offset, size_t len)
{
    const struct flash_s32k3_config *config = dev->config;
    struct flash_s32k3_data *data = dev->data;
    // C40_Ip_StatusType status;
    
    k_sem_take(&data->sem, K_FOREVER);
    
    /* Call C40 IP erase function */
    // status = C40_Ip_MainInterfaceSectorErase(/* params */);
    
    k_sem_give(&data->sem);
    
    return -EIO;
}

static int __ramfunc flash_s32k3_write(const struct device *dev,
                                      off_t offset,
                                      const void *data, size_t len)
{
    /* Implement C40 program operation */
    /* Must be aligned to 8-byte phrases */
    return (0);
}

static int flash_s32k3_read(const struct device *dev,
                          off_t offset, void *data, size_t len)
{
    const struct flash_s32k3_config *config = dev->config;

    memcpy(data, (void *)(config->base_addr + offset), len);
    return 0;
}

static const struct flash_parameters *
flash_s32k3_get_parameters(const struct device *dev)
{
    static struct flash_parameters params = {
        .write_block_size = DT_INST_PROP(0, write_block_size),
        .erase_value = 0xff,
    };
    return &params;
}

static const struct flash_driver_api flash_s32k3_api = {
    .erase = flash_s32k3_erase,
    .write = flash_s32k3_write,
    .read = flash_s32k3_read,
    .get_parameters = flash_s32k3_get_parameters,
    /* Add page_layout callback for MCUboot support */
};

static int flash_s32k3_init(const struct device *dev)
{
    struct flash_s32k3_data *data = dev->data;
    
    /* Initialize C40 IP driver */
    // C40_Ip_Init(&data->flash_config);
    
    k_sem_init(&data->sem, 1, 1);
    
    return 0;
}

#define FLASH_S32K3_DEVICE(inst)                                      \
    static const struct flash_s32k3_config flash_s32k3_config_##inst = { \
        .base_addr = DT_REG_ADDR(DT_INST_CHILD(inst, flash_400000)),      \
        .flash_size = DT_REG_SIZE(DT_INST_CHILD(inst, flash_400000)),     \
        .write_block_size = DT_INST_PROP(inst, write_block_size),   \
        .erase_block_size = DT_INST_PROP(inst, erase_block_size),   \
    };                                                              \
    static struct flash_s32k3_data flash_s32k3_data_##inst;            \
    DEVICE_DT_INST_DEFINE(inst, flash_s32k3_init, NULL,             \
                         &flash_s32k3_data_##inst,                   \
                         &flash_s32k3_config_##inst,                 \
                         POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,  \
                         &flash_s32k3_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_S32K3_DEVICE)
