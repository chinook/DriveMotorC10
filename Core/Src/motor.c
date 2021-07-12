/*
 * motor.c
 *
 *  Created on: Jul 10, 2021
 *      Author: Marc
 */

#include "motor.h"
#include "main.h"

enum DRIVE_PINS
{
	DRIVE_RESET,
	DRIVE_SLEEP,
	DRIVE_CS,
	DRIVE_STEP,
	DRIVE_DIR,

	NUM_PINS
};

GPIO_TypeDef* drive_ports[DRIVE_MOTOR_NUM][NUM_PINS] =
{
	{
		RESET1_GPIO_Port,
		nSLEEP1_GPIO_Port,
		SPI_CS1_GPIO_Port,
		STEP1_GPIO_Port,
		DIR1_GPIO_Port
	},
	{
		RESET2_GPIO_Port,
		nSLEEP2_GPIO_Port,
		SPI_CS2_GPIO_Port,
		STEP2_GPIO_Port,
		DIR2_GPIO_Port
	}
};
uint16_t drive_pins[DRIVE_MOTOR_NUM][NUM_PINS] =
{
	{
		RESET1_Pin,
		nSLEEP1_Pin,
		SPI_CS1_Pin,
		STEP1_Pin,
		DIR1_Pin
	},
	{
		RESET2_Pin,
		nSLEEP2_Pin,
		SPI_CS2_Pin,
		STEP2_Pin,
		DIR2_Pin
	}
};

// Helper SPI functions
static void TransmitSPI(SPI_HandleTypeDef* phspi, uint8_t reg, uint16_t data);
//static void ReceiveSPI(SPI_HandleTypeDef* phspi, uint8_t reg, uint16_t* data, uint16_t size);

void SendDriveRegister(SPI_HandleTypeDef* phspi, uint8_t reg, DRIVE_MOTOR drive_index);
static void SendDriveRegisters(SPI_HandleTypeDef* phspi, DRIVE_MOTOR drive_index);

static void InitRegValues(DRIVE_MOTOR drive_index);
static void InitDriveMoteur(SPI_HandleTypeDef* phspi, DRIVE_MOTOR drive_index);


// Private variables
DRV8711_REGS drive_regs[DRIVE_MOTOR_NUM];


void TransmitSPI(SPI_HandleTypeDef* phspi, uint8_t reg, uint16_t data)
{
	uint8_t tx_data[2];
	tx_data[1] = ((reg & 0x07) << 4) | ((data & 0xF00) >> 8);
	tx_data[0] = (data & 0xFF);

	HAL_SPI_Transmit(phspi, tx_data, 2, 100);
}

void SendDriveRegister(SPI_HandleTypeDef* phspi, uint8_t reg, DRIVE_MOTOR drive_index)
{
	// Can only send register from 0 to 6 (status register is read-only)
	if (reg >= 7)
		return;

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
						drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

	uint16_t data = *((uint16_t*)(&drive_regs[drive_index]) + reg);
	TransmitSPI(phspi, reg, data);

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
						drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
}

void SendDriveRegisters(SPI_HandleTypeDef* phspi, DRIVE_MOTOR drive_index)
{
	// Send every register to the drive
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

	// We send every register except for the status register (up to 7 register)
	for (uint8_t i = 0; i < (NUM_DRIVE_REGS-1); ++i)
	{
		uint16_t data = *((uint16_t*)(&drive_regs[drive_index]) + i);
		TransmitSPI(phspi, i, data);
	}

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
							drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
}

void InitRegValues(DRIVE_MOTOR drive_index)
{
	// CTRL register
	drive_regs[drive_index].ctrl_reg.enbl = 1;	// Enable motor
	drive_regs[drive_index].ctrl_reg.rdir = 0;	// Direction set by DIR pin
	drive_regs[drive_index].ctrl_reg.rstep = 0;	// No automatic stepping
	drive_regs[drive_index].ctrl_reg.mode = 0b0010;	// 1/4 step
	drive_regs[drive_index].ctrl_reg.extstall = 0;	// Internal stall detect
	drive_regs[drive_index].ctrl_reg.isgain = 0b00;	// Gain of 5
	drive_regs[drive_index].ctrl_reg.dtime = 0b11;	// Dead-time of 850ns

	// TORQUE register
	drive_regs[drive_index].torque_reg.torque = 0xFF;	// Sets full torque for H-bridge
	drive_regs[drive_index].torque_reg.smplth = 0b001;	// 100us BEMF sample threshold

	// OFF register
	drive_regs[drive_index].off_reg.toff = 0x30;		// Sets fixed off time, in increments of 500ns
	drive_regs[drive_index].off_reg.pwmmode = 0;		// Use internal indexer

	// BLANK register
	drive_regs[drive_index].blank_reg.tblank = 0x80;	// Current trip blanking time, in increments of 20ns
	drive_regs[drive_index].blank_reg.abt = 0;		// Disable adaptive blanking time

	// DECAY register
	drive_regs[drive_index].decay_reg.tdecay = 0x10;		// Mixed decay transition time, in increments of 500ns
	drive_regs[drive_index].decay_reg.decmod = 0b001;	// Slow decay for increasing current

	// STALL register
	drive_regs[drive_index].stall_reg.sdthr = 0x40;	// Stall detect threshold
	drive_regs[drive_index].stall_reg.sdcnt = 0b00;	// STALLn asserted on first step with BEMF below SDTHR
	drive_regs[drive_index].stall_reg.vdiv = 0b00;	// BEMF divided by 32

	// DRIVE register
	drive_regs[drive_index].drive_reg.ocpth = 0b01;		// OCP threshold of 500mV
	drive_regs[drive_index].drive_reg.ocpdeg = 0b10;		// OCP deglitch time of 4us
	drive_regs[drive_index].drive_reg.tdriven = 0b01;	// Low-side gate drive time of 500ns
	drive_regs[drive_index].drive_reg.tdrivep = 0b01;	// High-side gate drive time of 500ns
	drive_regs[drive_index].drive_reg.idriven = 0b10;	// Low-side gate drive peak current of 300mA peak (sink)
	drive_regs[drive_index].drive_reg.idrivep = 0b10;	// High-side gate drive peak current of 150mA peak (sink)
}

void InitDriveMoteur(SPI_HandleTypeDef* phspi, DRIVE_MOTOR drive_index)
{
	// Reset drive
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
			drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
			drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_RESET);

	// Disable sleeping
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_SLEEP],
			drive_pins[drive_index][DRIVE_SLEEP], GPIO_PIN_SET);

	// Set initial reg values
	InitRegValues(drive_index);

	// Send regs over SPI
	SendDriveRegisters(phspi, drive_index);
}


uint8_t InitDrivesMoteurs(SPI_HandleTypeDef* phspi)
{
	// Initialize both cs lines to up
	HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_CS],
			drive_pins[DRIVE1][DRIVE_CS], GPIO_PIN_SET);
	HAL_GPIO_WritePin(drive_ports[DRIVE2][DRIVE_CS],
				drive_pins[DRIVE2][DRIVE_CS], GPIO_PIN_SET);

	// Init both motor drives
	InitDriveMoteur(phspi, DRIVE1);
	InitDriveMoteur(phspi, DRIVE2);

	return 0;
}

void EnableDrive(SPI_HandleTypeDef* phspi, DRIVE_MOTOR drive_index)
{
	drive_regs[drive_index].ctrl_reg.enbl = 1;
	SendDriveRegister(phspi, DRV8711_CTRL_REG, drive_index);
}

void DisableDrive(SPI_HandleTypeDef* phspi, DRIVE_MOTOR drive_index)
{
	drive_regs[drive_index].ctrl_reg.enbl = 0;
	SendDriveRegister(phspi, DRV8711_CTRL_REG, drive_index);
}

void SetDirection(DRIVE_MOTOR drive_index, DRIVE_DIRECTION direction)
{
	GPIO_PinState state = (direction == FORWARD) ? 1 : 0;
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_DIR],
					drive_pins[drive_index][DRIVE_DIR], state);
}

void Step(DRIVE_MOTOR drive_index)
{
	HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_CS],
				drive_pins[DRIVE1][DRIVE_CS], GPIO_PIN_SET);
	for (int i = 0; i < 0xFFF; ++i);
	HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_CS],
				drive_pins[DRIVE1][DRIVE_CS], GPIO_PIN_RESET);
}

