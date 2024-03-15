#include <zephyr/device.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/util.h>
#define DT_DRV_COMPAT neorv32_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_neorv32);

#include <zephyr/drivers/spi.h>
#include <soc.h>
#include <errno.h>

#include "spi_neorv32.h"

static int spi_set_frequency(const struct device *dev, uint32_t frequency,
			     uint16_t *out_prsc, uint16_t *out_cdiv)
{
	static const uint16_t prescaler[] = {
		2,
		4,
		8,
		64,
		128,
		1024,
		2048,
		4096
	};

	struct spi_neorv32_data *data = dev->data;
	uint16_t prsc;
	uint16_t cdiv;
	uint32_t effective_frequency;

	for (prsc = 0; prsc < ARRAY_SIZE(prescaler); prsc++) {
		// Lookup with highest possible CDIV
		if (data->clk / 2 / prescaler[prsc] / 16 <= frequency)
			break;
	}

	if (prsc > ARRAY_SIZE(prescaler)) {
		LOG_ERR("Unsupported frequency %uHz, max %uHz, min %uHz",
			frequency, data->clk / 4, data->clk / 131072);
		return -EINVAL;
	}

	// Find the best CDIV value
	for (cdiv = 0; cdiv < 16; cdiv++) {
		effective_frequency = data->clk / 2 / prescaler[prsc] / (cdiv + 1);
		if (effective_frequency <= frequency)
			break;
	}

	LOG_DBG("SPI frequency %uHz, prsc %u, cdiv %u", effective_frequency,
		prsc, cdiv);

	*out_prsc = prsc;
	*out_cdiv = cdiv;

	return 0;
}

static void spi_neorv32_cs_control(const struct device *dev, bool on) {
	struct spi_neorv32_data *data = dev->data;
	const struct spi_neorv32_config *config = dev->config;
	uint32_t ctrl;

	ctrl = sys_read32(config->base + NEORV32_SPI_CTRL_OFFSET);
	if (on)
		ctrl |= NEORV32_SPI_CTRL_CS_EN;
	else
		ctrl &= ~NEORV32_SPI_CTRL_CS_EN;
	sys_write32(ctrl, config->base + NEORV32_SPI_CTRL_OFFSET);

	spi_context_cs_control(&data->ctx, on);
}

static void spi_wait_idle(const struct device *dev) {
	const struct spi_neorv32_config *config = dev->config;
	uint32_t ctrl;

	while (true) {
		ctrl = sys_read32(config->base + NEORV32_SPI_CTRL_OFFSET);
		if (!(ctrl & NEORV32_SPI_CTRL_BUSY))
			break;
	}
}

static int spi_configure(const struct device *dev,
			 const struct spi_config *config)
{
	struct spi_neorv32_data *data = dev->data;
	const struct spi_neorv32_config *nrv32_config = dev->config;
	int ret;
	uint16_t prsc;
	uint16_t cdiv;
	uint32_t ctrl;

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		return -ENOTSUP;
	}

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Only master mode is supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_FRAME_FORMAT_TI ||
		config->operation & SPI_TRANSFER_LSB ||
		config->operation & SPI_CS_ACTIVE_HIGH ||
		config->operation & SPI_MODE_LOOP ||
		(config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Unsupported operation mode request, operation 0x%x", config->operation);
		return -ENOTSUP;
	}

	if (config->slave > 7) {
		LOG_ERR("Maximum 8 slave devices are supported");
		return -EINVAL;
	}

	ret = spi_set_frequency(dev, config->frequency, &prsc, &cdiv);
	if (ret < 0)
		return ret;

	ctrl = NEORV32_SPI_CTRL_EN | (prsc << NEORV32_SPI_CTRL_PRSC_POS) |
		(cdiv << NEORV32_SPI_CTRL_CDIV_POS);

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL)
		ctrl |= NEORV32_SPI_CTRL_CPOL;

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA)
		ctrl |= NEORV32_SPI_CTRL_CPHA;

	if (!spi_cs_is_gpio(config)) {
		ctrl |= config->slave << NEORV32_SPI_CTRL_CS_POS;
		ctrl |= NEORV32_SPI_CTRL_CS_EN;
	}

	sys_write32(ctrl, nrv32_config->base + NEORV32_SPI_CTRL_OFFSET);
	/* Enable GPIO CS */
	spi_context_cs_control(&data->ctx, true);

	data->ctx.config = config;
	return 0;
}

static void shift_frame(const struct device *dev)
{
	const struct spi_neorv32_config *config = dev->config;
	struct spi_neorv32_data *data = dev->data;
	uint8_t temp;

	if (spi_context_tx_buf_on(&data->ctx)) {
		temp = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
	} else {
		// Set MOSI low when not transmitting anything.
		temp = 0x00;
	}

	while (sys_read32(config->base + NEORV32_SPI_CTRL_OFFSET)
		& NEORV32_SPI_CTRL_TXF) {}

	sys_write32((uint32_t)temp,
		    config->base + NEORV32_SPI_DATA_OFFSET);

	/* The update is ignored if TX is off. */
	spi_context_update_tx(&data->ctx, 1, 1);

	while ((sys_read32(config->base + NEORV32_SPI_CTRL_OFFSET)
		& NEORV32_SPI_CTRL_RXNE) == 0) {}

	temp = (uint8_t)sys_read32(config->base + NEORV32_SPI_DATA_OFFSET);

	if (spi_context_rx_buf_on(&data->ctx)) {
		UNALIGNED_PUT(temp, (uint8_t *)data->ctx.rx_buf);
	}

	spi_context_update_rx(&data->ctx, 1, 1);
}

static int transceive(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata)
{
	int ret = 0;
	struct spi_neorv32_data *data = dev->data;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	if (asynchronous) {
		return -ENOTSUP;
	}

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, config);

	/* Configure SPI, setting frequency, CPHA/CPOL and asserting CS# */
	if (!spi_context_configured(&data->ctx, config)) {
		ret = spi_configure(dev, config);
		if (ret < 0)
			goto end;
	} else {
		spi_neorv32_cs_control(dev, true);
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	/* Transfer data */
	do {
		shift_frame(dev);
	} while(spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx));

	spi_wait_idle(dev);
	if (!(config->operation & SPI_HOLD_ON_CS)) {
		spi_neorv32_cs_control(dev, false);
	}

end:
	spi_context_release(&data->ctx, ret);
	return ret;
}

static int spi_neorv32_transceive(const struct device *dev,
				  const struct spi_config *config,
				  const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_neorv32_transceive_async(const struct device *dev,
				        const struct spi_config *config,
				        const struct spi_buf_set *tx_bufs,
				        const struct spi_buf_set *rx_bufs,
				        spi_callback_t cb,
				        void *userdata)
{
	return transceive(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif

static int spi_neorv32_release(const struct device *dev,
			       const struct spi_config *config)
{
	struct spi_neorv32_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api api_funcs = {
	.transceive = spi_neorv32_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_neorv32_transceive_async,
#endif
	.release = spi_neorv32_release
};

static int spi_neorv32_init(const struct device *dev)
{
	int err;
	const struct spi_neorv32_config *config = dev->config;
	struct spi_neorv32_data *data = dev->data;

	if (!device_is_ready(config->sysinfo)) {
		LOG_ERR("SYSINFO device not ready");
		return -ENODEV;
	}

	err = syscon_read_reg(config->sysinfo, NEORV32_SYSINFO_CLK, &data->clk);
	if (err < 0) {
		LOG_ERR("Could not read CPU frequency");
		return err;
	}

	// Bootloader may leave SPI enabled.
	sys_write32(0, config->base + NEORV32_SPI_CTRL_OFFSET);

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define NEORV32_SPI_INIT(id)						\
static const struct spi_neorv32_config spi_neorv32_cfg_##id = {		\
	.base = DT_INST_REG_ADDR(id),					\
	.sysinfo = DEVICE_DT_GET(DT_INST_PHANDLE(id, syscon)),		\
};									\
static struct spi_neorv32_data spi_neorv32_data_##id = {		\
	.clk = 0,							\
	SPI_CONTEXT_INIT_LOCK(spi_neorv32_data_##id, ctx),		\
	SPI_CONTEXT_INIT_SYNC(spi_neorv32_data_##id, ctx),		\
};									\
									\
DEVICE_DT_INST_DEFINE(id, &spi_neorv32_init, NULL,			\
		      &spi_neorv32_data_##id, &spi_neorv32_cfg_##id,	\
		      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,		\
		      &api_funcs);


DT_INST_FOREACH_STATUS_OKAY(NEORV32_SPI_INIT)
