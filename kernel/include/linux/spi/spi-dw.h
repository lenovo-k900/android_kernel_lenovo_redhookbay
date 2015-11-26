#ifndef _SPI_DW_H_
#define _SPI_DW_H_

#define SPI_DW_DEASSERT	0
#define SPI_DW_ASSERT	1

#define	SPI_TMOD_TR			0x0		/* xmit & recv */
#define SPI_TMOD_TO			0x1		/* xmit only */
#define SPI_TMOD_RO			0x2		/* recv only */
#define SPI_TMOD_EPROMREAD		0x3		/* eeprom read mode */

/*
 * If the platform does not use on of the chip select lines provided
 * by the SOC it may register a chip select control function. The
 * address of initialized spi_dw_chip stucture in the controller_data
 * member of spi_board_info structure registered with the subsystem.
 */
struct dw_spi_chip {
        u8 tmode;
        u8 poll_mode;
        u8 type;
        u8 enable_dma;
	void (*cs_control)(u32 command);
};

#endif /* _SPI_DW_H_ */
