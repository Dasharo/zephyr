#ifndef ZEPHYR_DRIVERS_SPI_SPI_NEORV32_H_
#define ZEPHYR_DRIVERS_SPI_SPI_NEORV32_H_

#include "spi_context.h"
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/sys_io.h>

#define NEORV32_SPI_CTRL_OFFSET 0x00
#define NEORV32_SPI_DATA_OFFSET 0x04

#define NEORV32_SPI_CTRL_EN		BIT(0)
#define NEORV32_SPI_CTRL_CPHA		BIT(1)
#define NEORV32_SPI_CTRL_CPOL		BIT(2)
#define NEORV32_SPI_CTRL_CS_POS		3U
#define NEORV32_SPI_CTRL_CS_EN		BIT(6)
#define NEORV32_SPI_CTRL_PRSC_POS	7U
#define NEORV32_SPI_CTRL_PRSC_MASK	BIT_MASK(3)
#define NEORV32_SPI_CTRL_CDIV_POS	10U
#define NEORV32_SPI_CTRL_CDIV_MASK	BIT_MASK(4)
#define NEORV32_SPI_CTRL_RXNE		BIT(16)
#define NEORV32_SPI_CTRL_TXE		BIT(17)
#define NEORV32_SPI_CTRL_TXNHALF	BIT(18)
#define NEORV32_SPI_CTRL_TXF		BIT(19)
#define NEORV32_SPI_CTRL_IRQ_RXNE	BIT(20)
#define NEORV32_SPI_CTRL_IRQ_TXE	BIT(21)
#define NEORV32_SPI_CTRL_IRQ_TXNHALF	BIT(22)
#define NEORV32_SPI_CTRL_FIFO_LSB	BIT(23)
#define NEORV32_SPI_CTRL_FIFO_MSB	BIT(26)
#define NEORV32_SPI_CTRL_BUSY		BIT(31)

struct spi_neorv32_config {
	mem_addr_t base;
	const struct device *sysinfo;
};

struct spi_neorv32_data {
	uint32_t clk;
	struct spi_context ctx;
};

#endif /* ZEPHYR_DRIVERS_SPI_SPI_NEORV32_H_ */
