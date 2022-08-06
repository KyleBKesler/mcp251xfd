/*
 * mcp251xfd device control (using spidev driver)
 *
 * Copyright (c) 2021 TractorCloud, Inc
 * Copyright (c) 2021 Kyle Kesler <kyle@tractorcloud.io
 *
 * -- use case notes --
 *
 *
 */

#include "mcp251xfd_ctrl.h"
#include <time.h>
#include "../bcm2835-1.71/src/bcm2835.h"


// *************************************************************************************************
// *************************************************************************************************
// Setion: SPI Access Functions

char spiTransmitBuffer[SPI_DEFAULT_BUFFER_LENGTH];
char spiReceiveBuffer[SPI_DEFAULT_BUFFER_LENGTH];


int8_t mcp251xfd_spi_init(const uint8_t rpi_model, uint32_t speed_hz)
{
	uint32_t core_clk_freq;

	if (!bcm2835_init()) {
		fprintf(stderr, "bcm2835_init failed. Are you running as root?\n");
		return 1;
	}

	if (!bcm2835_spi_begin()) {
		fprintf(stderr, "bcm2835_spi_begin failed. Are you running as root?\n");
		return 1;
	}

	// Set core freq based on which processor
	switch (rpi_model) {
        	case 0:
			core_clk_freq = RPI0_CORE_CLK_FREQ;
			break;
		case 1:
			core_clk_freq = RPI3_CORE_CLK_FREQ;
			break;
		case 2:
			core_clk_freq = RPI4_CORE_CLK_FREQ;
			break;
		default:
			fprintf(stderr, "Invalid raspberry pi model.\n");
			return 1;
	}

	// Set clock divider value
	uint16_t divider = (uint16_t) (core_clk_freq / speed_hz);
	divider &= 0xFFFE;

	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
	bcm2835_spi_setClockDivider(divider);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

	return 0;
}


void mcp251xfd_spi_close(void)
{
	bcm2835_spi_end();
	bcm2835_close();
}


static void spi_read_byte(const uint8_t chipsel, uint16_t address, uint8_t *rxd)
{
	uint32_t transferSize = 3;

	bcm2835_spi_chipSelect(chipsel);

	// Compose Command
	spiTransmitBuffer[0] = (uint8_t) ((SPI_INSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t) (address & 0xFF);
	spiTransmitBuffer[2] = 0;

	bcm2835_spi_transfernb(&spiTransmitBuffer[0], &spiReceiveBuffer[0], transferSize);
	*rxd = (uint8_t) spiReceiveBuffer[2];
}


static void spi_write_byte(const uint8_t chipsel, uint16_t address, uint8_t txd)
{
	uint32_t transferSize = 3;

	bcm2835_spi_chipSelect(chipsel);

	// Compose Command
	spiTransmitBuffer[0] = (uint8_t) ((SPI_INSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t) (address & 0xFF);
	spiTransmitBuffer[2] = txd;

	bcm2835_spi_transfernb(&spiTransmitBuffer[0], &spiReceiveBuffer[0], transferSize);
}


static void spi_read_word(const uint8_t chipsel, uint16_t address, uint32_t *rxd)
{
	uint8_t i;
	uint32_t x;
	uint16_t transferSize = 6;

	bcm2835_spi_chipSelect(chipsel);

	// Compose command
	spiTransmitBuffer[0] = (uint8_t) ((SPI_INSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t) (address & 0xFF);

	bcm2835_spi_transfernb(&spiTransmitBuffer[0], &spiReceiveBuffer[0], transferSize);

	// Update data
	*rxd = 0;
	for (i = 2; i < 6; i++) {
		x = (uint32_t) spiReceiveBuffer[i];
		*rxd += x << ((i - 2) * 8);
	}
}


static void spi_write_word(const uint8_t chipsel, uint16_t address, uint32_t txd)
{
	uint8_t i;
	uint16_t transferSize = 6;

	bcm2835_spi_chipSelect(chipsel);

	// Compose command
	spiTransmitBuffer[0] = (uint8_t) ((SPI_INSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t) (address & 0xFF);

	// Split word into 4 bytes and add them to buffer
	for (i = 0; i < 4; i++) {
		spiTransmitBuffer[i + 2] = (uint8_t) ((txd >> (i * 8)) & 0xFF);
	}

	bcm2835_spi_transfernb(&spiTransmitBuffer[0], &spiReceiveBuffer[0], transferSize);
}


// *************************************************************************************************
// *************************************************************************************************
// Section: MCP251xFD Operation Mode Controls

const char *get_operation_mode_str(const uint8_t mode)
{
	switch (mode) {
		case MCP251XFD_NORMAL_MODE:
			return "Mixed (CAN FD/CAN2.0)";
		case MCP251XFD_SLEEP_MODE:
			return "Sleep";
		case MCP251XFD_INTERNAL_LOOPBACK_MODE:
			return "Internal Loopback";
		case MCP251XFD_LISTEN_ONLY_MODE:
			return "Listen Only";
		case MCP251XFD_CONFIGURATION_MODE:
			return "Configuration";
		case MCP251XFD_EXTERNAL_LOOPBACK_MODE:
			return "Externel Loopback";
		case MCP251XFD_CLASSIC_MODE:
			return "CAN 2.0";
		case MCP251XFD_RESTRICTED_MODE:
			return "Restricted Operation";
		/* add case for MCP2518FD low power mode */
	}

	return "<unknown>";
}


void get_operation_mode(const uint8_t chipsel, uint8_t *mode)
{
	uint8_t opmod_val;

	spi_read_byte(chipsel, MCP251XFD_REG_C1CON + MCP251XFD_C1CON_OPMOD, &opmod_val);

	*mode = GRAB_BITS(opmod_val, MCP251XFD_C1CON_OPMOD_POS, THREE_BIT_MASK);
}


static void set_operation_mode(const uint8_t chipsel, const uint8_t mode_req)
{
	uint8_t reqop_val;

	spi_read_byte(chipsel, MCP251XFD_REG_C1CON + MCP251XFD_C1CON_REQOP, &reqop_val);

	reqop_val &= ~THREE_BIT_MASK;
	reqop_val |= mode_req;

	spi_write_byte(chipsel, MCP251XFD_REG_C1CON + MCP251XFD_C1CON_REQOP, reqop_val);
}


static int8_t set_normal_mode_nowait(const uint8_t chipsel)
{
	uint8_t mode;

	set_operation_mode(chipsel, MCP251XFD_NORMAL_MODE);

	get_operation_mode(chipsel, &mode);

	if (mode) {
		fprintf(stderr, "Set normal mode failed. Current mode: %s\n", get_operation_mode_str(mode));
		return -1;
	}

	return 0;
}


static int8_t set_normal_mode_wait(const uint8_t chipsel)
{
	uint8_t mode = 1;
	time_t start_t, end_t;
	double timeout = 0.1;

	time(&start_t);
	while (mode && (difftime(end_t, start_t) < timeout)) {
		set_operation_mode(chipsel, MCP251XFD_NORMAL_MODE);

		get_operation_mode(chipsel, &mode);
		time(&end_t);
	}

    if (mode) {
        fprintf(stderr, "Set normal mode failed. Current mode: %s\n", get_operation_mode_str(mode));
		return -1;
	}

	return 0;
}


int8_t set_normal_mode(const uint8_t chipsel, bool nowait)
{
	if (nowait)
		return set_normal_mode_nowait(chipsel);

	return set_normal_mode_wait(chipsel);
}


static int8_t sleep_mode_handshake(const uint8_t chipsel, uint8_t opmod_val)
{
	uint8_t osc_byte1;

	get_operation_mode(chipsel, &opmod_val);
	spi_read_byte(chipsel, MCP251XFD_REG_OSC, &osc_byte1);

	osc_byte1 &= MCP251XFD_OSC_OSCDIS_VAL;
	if ((opmod_val == MCP251XFD_CONFIGURATION_MODE) && osc_byte1) {
		return 0;
	}

	return -1;
}


static int8_t set_sleep_mode_nowait(const uint8_t chipsel)
{
	uint8_t mode = 0xFF;
	uint8_t osc_byte1;

	// Clear OSC.LPMEN Bit
	spi_read_byte(chipsel, MCP251XFD_REG_OSC, &osc_byte1);
	osc_byte1 &= ~MCP251XFD_OSC_LPMEN_VAL;
	spi_write_byte(chipsel, MCP251XFD_REG_OSC, osc_byte1);

	set_operation_mode(chipsel, MCP251XFD_SLEEP_MODE);

	if (sleep_mode_handshake(chipsel, mode)) {
		fprintf(stderr, "Set sleep mode failed. Current mode: %s\n", get_operation_mode_str(mode));
		return -1;
	}

	return 0;
}

static int8_t set_sleep_mode_wait(const uint8_t chipsel)
{
	uint8_t mode = 0xFF;
	uint8_t osc_byte1;
	time_t start_t, end_t;
	double timeout = 0.1;

	time(&start_t);
	while (sleep_mode_handshake(chipsel, mode) && (difftime(end_t, start_t) < timeout)) {
    	// Clear OSC.LPMEN Bit
    	spi_read_byte(chipsel, MCP251XFD_REG_OSC, &osc_byte1);
		osc_byte1 &= ~MCP251XFD_OSC_LPMEN_VAL;
		spi_write_byte(chipsel, MCP251XFD_REG_OSC, osc_byte1);

    	set_operation_mode(chipsel, MCP251XFD_SLEEP_MODE);
		time(&end_t);
	}

    if (sleep_mode_handshake(chipsel, mode)) {
        fprintf(stderr, "Set sleep mode failed. Current mode: %s\n", get_operation_mode_str(mode));
        return -1;
	}

    return 0;
}


int8_t set_sleep_mode(const uint8_t chipsel, bool nowait)
{
	if (nowait)
		return set_sleep_mode_nowait(chipsel);

	return set_sleep_mode_wait(chipsel);
}


void exit_sleep_mode(const uint8_t chipsel)
{
	uint8_t osc_byte1;

	// Clear OSC.OSCDIS Bit
	spi_read_byte(chipsel, MCP251XFD_REG_OSC, &osc_byte1);
	osc_byte1 &= ~MCP251XFD_OSC_OSCDIS_VAL;
	spi_write_byte(chipsel, MCP251XFD_REG_OSC, osc_byte1);
}


static int8_t set_configuration_mode_nowait(const uint8_t chipsel)
{
	uint8_t mode;

	set_operation_mode(chipsel, MCP251XFD_CONFIGURATION_MODE);

	get_operation_mode(chipsel, &mode);

    if (mode != MCP251XFD_CONFIGURATION_MODE) {
        fprintf(stderr, "Set config mode failed. Current mode: %s\n", get_operation_mode_str(mode));
        return -1;
    }

    return 0;
}


static int8_t set_configuration_mode_wait(const uint8_t chipsel)
{
	uint8_t mode = 1;
    time_t start_t, end_t;
    double timeout = 0.1;

    time(&start_t);
    while ((mode != MCP251XFD_CONFIGURATION_MODE) && (difftime(end_t, start_t) < timeout)) {
        set_operation_mode(chipsel, MCP251XFD_CONFIGURATION_MODE);

        get_operation_mode(chipsel, &mode);
    	time(&end_t);
	}

    if (mode != MCP251XFD_CONFIGURATION_MODE) {
        fprintf(stderr, "Set config mode failed. Current mode: %s\n", get_operation_mode_str(mode));
        return -1;
    }

    return 0;
}


int8_t set_configuration_mode(const uint8_t chipsel, bool nowait)
{
	if (nowait)
		return set_configuration_mode_nowait(chipsel);

	return set_configuration_mode_wait(chipsel);
}


/* Add Low Power Mode (LPM) for MCP2518FD */
/*

int8_t set_low_power_mode() {}

*/

// *************************************************************************************************
// *************************************************************************************************
// Section: Wakeup Controls

void wake_filter_on_config(const uint8_t chipsel, uint8_t filter_num)
{
	uint8_t wakfil;
	uint8_t wft;

	if (filter_num > 3) {
		filter_num = 0;
	}

	spi_read_byte(chipsel, MCP251XFD_REG_C1CON + MCP251XFD_C1CON_WAKFIL, &wakfil);

	wft = filter_num << 1;
	wakfil |= (wft + MCP251XFD_C1CON_WAKFIL_VAL);

	spi_write_byte(chipsel, MCP251XFD_REG_C1CON + MCP251XFD_C1CON_WAKFIL, wakfil);
}


void wake_filter_off(const uint8_t chipsel)
{
	uint8_t wakfil;

	spi_read_byte(chipsel, MCP251XFD_REG_C1CON + MCP251XFD_C1CON_WAKFIL, &wakfil);

	wakfil &= ~(MCP251XFD_C1CON_WFT_VAL + MCP251XFD_C1CON_WAKFIL_VAL);
	spi_write_byte(chipsel, MCP251XFD_REG_C1CON + MCP251XFD_C1CON_WAKFIL, wakfil);
}


void wakeup_interrupt_enable(const uint8_t chipsel)
{
	uint8_t wakie;

	// Set WAKIE Bit
	spi_read_byte(chipsel, MCP251XFD_REG_C1INT + MCP251XFD_C1INT_WAKIE, &wakie);

	wakie |= MCP251XFD_C1INT_WAKIE_VAL;
	spi_write_byte(chipsel, MCP251XFD_REG_C1INT + MCP251XFD_C1INT_WAKIE, wakie);
}


void wakeup_interrupt_disable(const uint8_t chipsel)
{
	uint8_t wakie;

	// Clear WAKIE Bit
	spi_read_byte(chipsel, MCP251XFD_REG_C1INT + MCP251XFD_C1INT_WAKIE, &wakie);

	wakie &= ~MCP251XFD_C1INT_WAKIE_VAL;
	spi_write_byte(chipsel, MCP251XFD_REG_C1INT + MCP251XFD_C1INT_WAKIE, wakie);
}


// *************************************************************************************************
// *************************************************************************************************
// Section: INT/GPIO/STDBY Control

void enable_transceiver_stdby_ctrl(const uint8_t chipsel)
{
	uint8_t xstbyen;

	// Set XSTBYEN Bit
	spi_read_byte(chipsel, MCP251XFD_REG_IOCON, &xstbyen);
	xstbyen |= MCP251XFD_IOCON_XSTBYEN_VAL;
	spi_write_byte(chipsel, MCP251XFD_REG_IOCON, xstbyen);
}


void disable_transceiver_stdby_ctrl(const uint8_t chipsel)
{
	uint8_t xstbyen;

    // Clear XSTBYEN Bit
    spi_read_byte(chipsel, MCP251XFD_REG_IOCON, &xstbyen);
    xstbyen &= ~MCP251XFD_IOCON_XSTBYEN_VAL;
    spi_write_byte(chipsel, MCP251XFD_REG_IOCON, xstbyen);
}


void int0_gpio0_ctrl(const uint8_t chipsel, const uint8_t mode)
{
	uint8_t pm0;

    // Set XSTBYEN Bit
    spi_read_byte(chipsel, MCP251XFD_REG_IOCON + MCP251XFD_IOCON_PM0, &pm0);

	if (mode) {
		// Set as GPIO Pin
		pm0 |= MCP251XFD_IOCON_PM0_VAL;
	}
	else {
		// Set as Interrupt Pin
		pm0 &= ~MCP251XFD_IOCON_PM0_VAL;
    }

	spi_write_byte(chipsel, MCP251XFD_REG_IOCON + MCP251XFD_IOCON_PM0, pm0);
}


void int1_gpio1_ctrl(const uint8_t chipsel, const uint8_t mode)
{
	uint8_t pm1;

    // Set XSTBYEN Bit
    spi_read_byte(chipsel, MCP251XFD_REG_IOCON + MCP251XFD_IOCON_PM1, &pm1);

    if (mode) {
		// Set as GPIO Pin
        pm1 |= MCP251XFD_IOCON_PM1_VAL;
    }
    else {
		// Set as Interrupt Pin
        pm1 &= ~MCP251XFD_IOCON_PM1_VAL;
    }

    spi_write_byte(chipsel, MCP251XFD_REG_IOCON + MCP251XFD_IOCON_PM1, pm1);
}


void gpio0_set_latch(const uint8_t chipsel, const uint8_t mode)
{
	uint8_t lat0;

	// Set LAT0 Bit
	spi_read_byte(chipsel, MCP251XFD_REG_IOCON + MCP251XFD_IOCON_LAT1, &lat0);

	if (mode) {
		// Drive Pin High
		lat0 |= MCP251XFD_IOCON_LAT0_VAL;
	}
	else {
		// Drive Pin Low
		lat0 &= MCP251XFD_IOCON_LAT0_VAL;
	}

	spi_write_byte(chipsel, MCP251XFD_REG_IOCON + MCP251XFD_IOCON_LAT0, lat0);
}


void gpio1_set_latch(const uint8_t chipsel, const uint8_t mode)
{
	uint8_t lat1;

	// Set LAT1 Bit
	spi_read_byte(chipsel, MCP251XFD_REG_IOCON + MCP251XFD_IOCON_LAT1, &lat1);

	if (mode) {
		// Drive Pin High
		lat1 |= MCP251XFD_IOCON_LAT1_VAL;
	}
	else {
		// Drive Pin Low
		lat1 &= MCP251XFD_IOCON_LAT1_VAL;
	}

	spi_write_byte(chipsel, MCP251XFD_REG_IOCON + MCP251XFD_IOCON_LAT1, lat1);
}


void set_gpio0_direction(const uint8_t chipsel, const uint8_t mode)
{
	uint8_t tris0;

    // Set TRIS0 Bit
    spi_read_byte(chipsel, MCP251XFD_REG_IOCON, &tris0);

    if (mode) {
        // Input Pin
        tris0 |= MCP251XFD_IOCON_TRIS0_VAL;
    }
    else {
        // Output Pin
        tris0 &= MCP251XFD_IOCON_TRIS0_VAL;
    }

    spi_write_byte(chipsel, MCP251XFD_REG_IOCON, tris0);
}


void set_gpio1_direction(const uint8_t chipsel, const uint8_t mode)
{
	uint8_t tris1;

    // Set TRIS1 Bit
    spi_read_byte(chipsel, MCP251XFD_REG_IOCON, &tris1);

    if (mode) {
        // Input Pin
        tris1 |= MCP251XFD_IOCON_TRIS1_VAL;
    }
    else {
        // Output Pin
        tris1 &= MCP251XFD_IOCON_TRIS1_VAL;
    }

    spi_write_byte(chipsel, MCP251XFD_REG_IOCON, tris1);
}

