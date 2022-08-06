/*
 * mcp251xfd device control (using spidev driver)
 *
 * Copyright (c) 2021 TractorCloud, Inc
 * Copyright (c) 2021 Kyle Kesler <kyle@tractorcloud.io>
 *
 */

#ifndef MCP251XFD_CTRL_H
#define MCP251XFD_CTRL_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//* Bit Operator Macros *//
#define GRAB_BITS(x, y, z) ((x >> y) & z)
#define THREE_BIT_MASK 0x07


//* MCP251x Registers *//

/* CAN FD Controller Module SFR */
#define MCP251XFD_REG_C1CON 		0x000
#define MCP251XFD_C1CON_OPMOD		0x002	/* Byte Position */
#define MCP251XFD_C1CON_OPMOD_POS 	0x05	/* Bit Position */
#define MCP251XFD_C1CON_REQOP		0x003
#define MCP251XFD_C1CON_REQOP_POS 	0x00
#define MCP251XFD_C1CON_WAKFIL		0x001
#define MCP251XFD_C1CON_WAKFIL_VAL	0x01	/* Bit Position Value */
#define MCP251XFD_C1CON_WFT_VAL		0x06	/* ex) bit3=0x4, Bit2,3=0x6 */

#define MCP251XFD_REG_C1NBTCFG 		0x004
#define MCP251XFD_REG_C1DBTCFG 		0x008
#define MCP251XFD_REG_C1TDC 		0x00C
#define MCP251XFD_REG_C1TBC 		0x010
#define MCP251XFD_REG_C1TSCON 		0x014
#define MCP251XFD_REG_C1VEC 		0x018

#define MCP251XFD_REG_C1INT 		0x01C
#define MCP251XFD_C1INT_WAKIE		0x003
#define MCP251XFD_C1INT_WAKIE_VAL 	0x40

#define MCP251XFD_REG_C1RXIF 		0x020
#define MCP251XFD_REG_C1TXIF 		0x024
#define MCP251XFD_REG_C1RXOVIF 		0x028
#define MCP251XFD_REG_C1TXATIF 		0x02C
#define MCP251XFD_REG_C1TXREQ 		0x030
#define MCP251XFD_REG_C1TREC 		0x034
#define MCP251XFD_REG_C1BDIAG0 		0x038
#define MCP251XFD_REG_C1BDIAG1 		0x03C
#define MCP251XFD_REG_C1TEFCON 		0x040
#define MCP251XFD_REG_C1TEFSTA 		0x044
#define MCP251XFD_REG_C1TEFUA 		0x048
#define MCP251XFD_REG_C1TXQCON 		0x050
#define MCP251XFD_REG_C1TXQSTA 		0x054
#define MCP251XFD_REG_C1TXQUA 		0x058
#define MCP251XFD_REG_C1FIFOCON(x) 	(0x50 + 0xC * (x))
#define MCP251XFD_REG_C1FIFOSTA(x) 	(0x54 + 0xC * (x))
#define MCP251XFD_REG_C1FIFOUA(x) 	(0x58 + 0xC * (x))
#define MCP251XFD_REG_C1FLTCON(x) 	(0x1D0 + 0x4 * (x))
#define MCP251XFD_REG_C1FLTOBJ(x) 	(0x1F0 + 0x8 * (x))


//* MCP2517/18FD SFR *//

/* Oscillator Control */
#define MCP251XFD_REG_OSC 			0xE00
#define MCP251XFD_OSC_LPMEN_VAL		0x08
#define MCP251XFD_OSC_OSCDIS_VAL	0x04

/* IO Control */
#define MCP251XFD_REG_IOCON 		0xE04
#define MCP251XFD_IOCON_XSTBYEN_VAL	0x40
#define MCP251XFD_IOCON_TRIS0_VAL	0x01
#define MCP251XFD_IOCON_TRIS1_VAL	0x02
#define MCP251XFD_IOCON_LAT0		0x001
#define MCP251XFD_IOCON_LAT0_VAL	0x01
#define MCP251XFD_IOCON_LAT1		0x001
#define MCP251XFD_IOCON_LAT1_VAL	0x02
#define MCP251XFD_IOCON_PM0			0x003
#define MCP251XFD_IOCON_PM0_VAL		0x01
#define MCP251XFD_IOCON_PM1			0x003
#define MCP251XFD_IOCON_PM1_VAL		0x02

#define MCP251XFD_REG_CRC 			0xE08
#define MCP251XFD_REG_ECCCON 		0xE0C
#define MCP251XFD_REG_ECCSTAT 		0xE10


//* MCP251x Operation Modes *//
#define MCP251XFD_NORMAL_MODE               0x00
#define MCP251XFD_SLEEP_MODE                0x01
#define MCP251XFD_INTERNAL_LOOPBACK_MODE    0x02
#define MCP251XFD_LISTEN_ONLY_MODE          0x03
#define MCP251XFD_CONFIGURATION_MODE        0x04
#define MCP251XFD_EXTERNAL_LOOPBACK_MODE    0x05
#define MCP251XFD_CLASSIC_MODE              0x06
#define MCP251XFD_RESTRICTED_MODE           0x07


//* SPI Instruction Set *//
#define SPI_INSTRUCTION_RESET		0x00
#define SPI_INSTRUCTION_WRITE		0x02
#define SPI_INSTRUCTION_READ		0x03
#define SPI_INSTRUCTION_WRITE_CRC	0x0A
#define SPI_INSTRUCTION_READ_CRC	0x0B
#define SPI_INSTRUCTION_WRITE_SAFE	0x0C

/* SPI CLOCK CONFIGURATION */
#define RPI0_CORE_CLK_FREQ 1000000000
#define RPI3_CORE_CLK_FREQ 1400000000
#define RPI4_CORE_CLK_FREQ 1500000000
#define MCP251XFD_SPICLOCK_HZ_MAX 20000000

/* SPI Defs */
#define SPI_DEFAULT_BUFFER_LENGTH 4096



/* Non-Static Functions */
int8_t mcp251xfd_spi_init(const uint8_t rpi_model, uint32_t speed_hz);

void mcp251xfd_spi_close(void);

const char *get_operation_mode_str(const uint8_t mode);

void get_operation_mode(const uint8_t chipsel, uint8_t *mode);

int8_t set_normal_mode(const uint8_t chipsel, bool nowait);

int8_t set_sleep_mode(const uint8_t chipsel, bool nowait);

void exit_sleep_mode(const uint8_t chipsel);

int8_t set_configuration_mode(const uint8_t chipsel, bool nowait);

void wake_filter_on_config(const uint8_t chipsel, uint8_t filter_num);

void wake_filter_off(const uint8_t chipsel);

void wakeup_interrupt_enable(const uint8_t chipsel);

void wakeup_interrupt_disable(const uint8_t chipsel);

void enable_transceiver_stdby_ctrl(const uint8_t chipsel);

void disable_transceiver_stdby_ctrl(const uint8_t chipsel);

void int0_gpio0_ctrl(const uint8_t chipsel, const uint8_t mode);

void int1_gpio1_ctrl(const uint8_t chipsel, const uint8_t mode);

void gpio0_set_latch(const uint8_t chipsel, const uint8_t mode);

void gpio1_set_latch(const uint8_t chipsel, const uint8_t mode);

void set_gpio0_direction(const uint8_t chipsel, const uint8_t mode);

void set_gpio1_direction(const uint8_t chipsel, const uint8_t mode);

#endif
