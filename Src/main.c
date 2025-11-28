#include "stm32f4xx.h"
#include <stdint.h>
#include "uart.h"
#include "systick.h"
#include <string.h>
#include "gpio.h"
#include <stdio.h>

#define FLASH_SECTOR2_BASE_ADDRESS (0x08008000)
#define MAX_BL_RX_BUFFER_LENGTH   255
#define VERIFY_CRC_SUCCESS 0
#define VERIFY_CRC_FAIL    1
#define BOOTLOADER_VERSION 0x10 // Version 1.0

#define SRAM1_SIZE				(112*1024)
#define SRAM1_END				(SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE				(16*1024)
#define SRAM2_END				(SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE				(1024*1024)
//#define FLASH_END				(FLASH_BASE + FLASH_SIZE)
#define BKPSRAM_SIZE			(4*1024)
#define BKPSRAM_END				(BKPSRAM_BASE + BKPSRAM_SIZE)

#define BL_GET_VER   			0x51
#define BL_GET_HELP  			0x52
#define BL_GET_CID  			0x53
#define BL_GET_RDP_STATUS 		0x54
#define BL_GO_TO_ADDR 	 		0x55
#define BL_FLASH_ERASE 			0x56
#define BL_MEM_WRITE 			0x57



// Define voltage range for flash operations
#define FLASH_VOLTAGE_RANGE_3 ((uint32_t)0x00000002U) // 2.7 to 3.6V



// Global buffer to accumulate command bytes
uint8_t bl_rx_buffer[MAX_BL_RX_BUFFER_LENGTH];
// Function prototypes for command handling
void bootloader_handle_get_ver(uint8_t *pBuffer);
void bootloader_handle_gethelp(uint8_t *pBuffer);
void bootloader_handle_getcid(uint8_t *pBuffer);
void bootloader_handle_getrdp(uint8_t *pBuffer);
void bootloader_handle_go_to_addr(uint8_t *pBuffer);
void bootloader_handle_flash_erase(uint8_t *pBuffer);
void bootloader_handle_mem_write (uint8_t *pBuffer);



void bootloader_uart_read_data(void);

uint32_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host);

uint8_t get_bootloader_version(void);

void bootloader_jump_to_user_app(void);

uint32_t uwCRCValue = 0xff; // Initial value

uint8_t supported_commands[] = {0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a};

int main(void)
{
	//uart3_init();
	uart_init();
	systick_msec_delay(1000);

	if(get_btn_state())
		bootloader_uart_read_data();
	else
		bootloader_jump_to_user_app();
}

void bootloader_jump_to_user_app(void)
{
    // Function pointer to the user application's reset handler
    void (*app_reset_handler)(void);

    // 1. Get the Main Stack Pointer (MSP) value from the user application's vector table
    // The MSP is the first 32-bit word at the user application's flash base address
    uint32_t msp_value = *((volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS);

    // 2. Set the MSP for the user application using CMSIS API
    __set_MSP(msp_value);

    // 3. Get the Reset Handler address from the user application's vector table
    // The Reset Handler address is the second 32-bit word (+4 bytes offset)
    uint32_t reset_handler_address = *((volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4));

    // 4. Set up the function pointer to the user application's Reset Handler
    app_reset_handler = (void(*)(void))reset_handler_address;

    // 5. Jump to the user application's Reset Handler
    app_reset_handler();
}

void bootloader_uart_read_data(void)
{
    uint8_t rcv_len = 0;

    // This loop constantly waits for commands
    while(1)
    {
        // Clear the buffer for a new command
        memset(bl_rx_buffer, 0, MAX_BL_RX_BUFFER_LENGTH);

        // 1. Read the first byte, which is the "length to follow"
        bl_rx_buffer[0] = uart_read();
        rcv_len=bl_rx_buffer[0];
        // 2. Read the remaining bytes into the buffer, starting from the second byte
        // The total number of bytes to read is rcv_len
        // The command code and payload are part of rcv_len
        for (uint8_t i = 0; i < rcv_len; i++)
        {
            bl_rx_buffer[i+1] = uart_read();
        }

        // 3. Decode the command code using a switch statement
        // The command code is the first byte of the data payload
        switch(bl_rx_buffer[1])
        {
            case BL_GET_VER:
                bootloader_handle_get_ver(bl_rx_buffer);
                break;
            // Add other command cases here
            case BL_GET_HELP:
            	bootloader_handle_gethelp(bl_rx_buffer);
                break;
            case BL_GET_CID:
            	bootloader_handle_getcid(bl_rx_buffer);
            	break;
            case BL_GET_RDP_STATUS:
            	bootloader_handle_getrdp(bl_rx_buffer);
            	break;
            case BL_GO_TO_ADDR:
            	bootloader_handle_go_to_addr(bl_rx_buffer);
            	break;
            case BL_FLASH_ERASE:
            	bootloader_handle_flash_erase(bl_rx_buffer);
            	break;
            case BL_MEM_WRITE:
            	bootloader_handle_mem_write(bl_rx_buffer);
            	break;
            default:
                // Handle invalid command
                // You can add code here to send a NACK or an error message
                break;
        }
    }
}

void bootloader_handle_get_ver(uint8_t *pBuffer)
{
    uint32_t command_packet_len = pBuffer[0]+1; // Total length of the command packet
    uint32_t crc_value = 0;
    uint32_t calculated_crc = 0;
    uint8_t bl_version = 0;

    // 1. Verify the CRC of the received command packet
    // The CRC to be verified is located at the end of the packet
    crc_value = *(uint32_t *)(pBuffer + command_packet_len - 4);

    // The data length for CRC calculation is the total length minus the 4 CRC bytes
    uint32_t data_len = command_packet_len - 4;

    calculated_crc = bootloader_verify_crc(pBuffer, data_len, crc_value);

    if (calculated_crc == VERIFY_CRC_SUCCESS)
    {
        // 2. Checksum is correct, so send an ACK
        // The reply for this cogmmand is 1 byte (the version number)
        bootloader_send_ack(1);

        // 3. Get the bootloader version
        bl_version = get_bootloader_version();

        // 4. Send the reply back to the host
        bootloader_uart_write_data(&bl_version, 1);
    }
    else
    {
        // 5. Checksum is incorrect, send a NACK
        bootloader_send_nack();
    }
}

void bootloader_handle_gethelp(uint8_t *pBuffer)
{
    uint32_t command_packet_len = pBuffer[0] + 1; // Total length of the command packet
    uint32_t crc_value = 0;
    uint32_t calculated_crc = 0;

    // 1. Verify the CRC of the received command packet
    // The CRC to be verified is located at the end of the packet
    crc_value = *((uint32_t *)(pBuffer + command_packet_len - 4));

    // The data length for CRC calculation is the total length minus the 4 CRC bytes
    uint32_t data_len = command_packet_len - 4;

    calculated_crc = bootloader_verify_crc(pBuffer, data_len, crc_value);

    if (calculated_crc == VERIFY_CRC_SUCCESS)
    {
        // 2. Checksum is correct, so send an ACK
        // The reply for this command is the list of supported commands
        bootloader_send_ack(sizeof(supported_commands));

        // 3. Send the reply back to the host, which is the array of supported commands
        bootloader_uart_write_data(supported_commands, sizeof(supported_commands));
    }
    else
    {
        // 4. Checksum is incorrect, send a NACK
        bootloader_send_nack();
    }
}

uint32_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t computed_crc = 0;
	    uint32_t i, data_word;

	    // Enable CRC peripheral clock (adjust depending on your STM32 family)
	    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

	    // Reset CRC calculation unit by writing to CR register
	    CRC->CR = 1;

		// Feed data to the CRC peripheral
		for (uint32_t i = 0; i < len; i++)
		{
			// Write the 32-bit data word to the CRC_DR register.
			// The CRC peripheral automatically processes the data.
			CRC->DR = pData[i];
			//printf("Processing data word 0x%08lX\r\n", data[i]);
		}

	    // Read the CRC result from DR register
	    computed_crc = CRC->DR;

	    // Disable CRC clock if desired (optional for power saving)
	    // RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;


    // 4. Compare the calculated CRC with the host's CRC
    if (computed_crc == crc_host)
    {
        return VERIFY_CRC_SUCCESS;
    }
    else
    {
        return VERIFY_CRC_FAIL;
    }
}



uint8_t get_bootloader_version(void)
{
    return BOOTLOADER_VERSION;
}


uint16_t get_mcu_chip_id(void)
{
    // DBGMCU is a structure defined in the header file,
    // which gives access to the Debug MCU registers.
    // The IDCODE register holds the chip ID.
    // We mask the value to get only bits 0-11, which represent the device identifier.
    return (uint16_t)(DBGMCU->IDCODE & 0x0FFF);
}

/* Bootloader command handler for BL_GET_CID */
void bootloader_handle_getcid(uint8_t *pBuffer)
{
    uint32_t command_packet_len = pBuffer[0] + 1;
    uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));

    if (bootloader_verify_crc(pBuffer, command_packet_len - 4, host_crc) == VERIFY_CRC_SUCCESS)
    {
        // Checksum is correct, send an ACK
        bootloader_send_ack(2); // The chip ID is 2 bytes (uint16_t)

        // Get the 2-byte chip ID
        uint16_t chip_id = get_mcu_chip_id();

        // Send the reply back to the host
        // The chip ID is 2 bytes, so we write them individually
        bootloader_uart_write_data((uint8_t *)&chip_id, sizeof(chip_id));
    }
    else
    {
        // Checksum is incorrect, send a NACK
        bootloader_send_nack();
    }
}

// Helper function to read the Flash Read Protection (RDP) level
uint8_t get_flash_rdp_level(void)
{
    // The RDP level is stored in the option byte at address 0x40023C10, specifically in bits 8-15.
    uint32_t *p_ob_address = (uint32_t *) 0x1FFFC000
;

    // Read the 32-bit value from the option byte address,
    // shift right by 8 bits to get the second byte, and mask it.
    uint8_t rdp_status = (uint8_t)((*p_ob_address >> 8) & 0xFF);

    return rdp_status;
}

// Bootloader command handler for BL_GET_RDP_STATUS
void bootloader_handle_getrdp(uint8_t *pBuffer)
{
    uint32_t command_packet_len = pBuffer[0] + 1;
    uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
    uint8_t rdp_level = 0;

    if (bootloader_verify_crc(pBuffer, command_packet_len - 4, host_crc) == VERIFY_CRC_SUCCESS)
    {
        // Checksum is correct, send an ACK
        bootloader_send_ack(1); // The RDP level is a single byte

        // Get the RDP level using the helper function
        rdp_level = get_flash_rdp_level();

        // Send the reply back to the host
        bootloader_uart_write_data(&rdp_level, 1);
    }
    else
    {
        // Checksum is incorrect, send a NACK
        bootloader_send_nack();
    }
}

int verify_address(uint32_t addr) {
    // Check if the address falls within the Flash memory region
    if ((addr >= FLASH_BASE) && (addr < (FLASH_END))) {
        return 1;
    }

    // Check if the address falls within the SRAM1 region
    if ((addr >= SRAM1_BASE) && (addr < SRAM1_END)) {
        return 1;
    }

    // Check if the address falls within the SRAM2 region
    if ((addr >= SRAM2_BASE) && (addr < SRAM2_END)) {
        return 1;
    }

    // Check if the address falls within the Backup SRAM (BKPSRAM) region
    if ((addr >= BKPSRAM_BASE) && (addr < BKPSRAM_END)) {
        return 1;
    }

    // If the address does not fall into any of the valid regions, it's considered invalid
    return 0;
}

void bootloader_handle_go_to_addr(uint8_t *pBuffer)
{
    // The command packet format is:
    // [Total length - 1] [Command code] [Address (4 bytes, little-endian)] [CRC (4 bytes)]

    uint32_t command_packet_len = pBuffer[0] + 1;
    uint32_t crc_value = 0;
    uint32_t calculated_crc = 0;
    uint32_t go_address = 0;

    // 1. Verify the CRC of the received command packet
    // The CRC to be verified is located at the end of the packet
    crc_value = *(uint32_t *)(pBuffer + command_packet_len - 4);
    uint32_t data_len = command_packet_len - 4;
    calculated_crc = bootloader_verify_crc(pBuffer, data_len, crc_value);

    if (calculated_crc == VERIFY_CRC_SUCCESS)
    {
        // Checksum is correct, send an ACK
        bootloader_send_ack(1);



        // 2. Extract the address from the buffer
        // The address bytes are at index 2 to 5 in the buffer (little-endian)
        go_address = *(uint32_t *)(pBuffer + 1);
        //go_address = 0x08008000;

        // 3. Verify the address before attempting to jump
        if (verify_address(go_address) == 1)
        {
            // Address is valid, proceed with the jump


            // Function pointer to the application's Reset Handler
            void (*app_reset_handler)(void);

            // Set the Main Stack Pointer (MSP)
            // The first entry in the vector table (at 'go_address') is the MSP
            __set_MSP(*((volatile uint32_t*)go_address));

            // Configure the Vector Table Offset Register (VTOR)
            // The VTOR must point to the new application's vector table
            SCB->VTOR = go_address;

            // Get the address of the application's Reset Handler
            // The second entry in the vector table (at 'go_address + 4') is the Reset Handler
            app_reset_handler = (void (*)(void))(*((volatile uint32_t*)(go_address + 4)));

            // Jump to the application's Reset Handler
            app_reset_handler();
        	//bootloader_jump_to_user_app();
        }
        else
        {
            // Address is not valid, send NACK
            bootloader_send_nack();
        }
    }
    else
    {
        // Checksum is incorrect, send a NACK
        bootloader_send_nack();
    }
}

// Function to unlock the Flash control register (FLASH_CR)
static void flash_unlock(void)
{
    // Unlock the FLASH_CR register to allow modifications.
    // This is done by writing two specific keys to the FLASH_KEYR register.
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        FLASH->KEYR = 0x45670123U; // Key 1
        FLASH->KEYR = 0xCDEF89ABU; // Key 2
    }
}

// Function to lock the Flash control register (FLASH_CR)
static void flash_lock(void)
{
    // Lock the FLASH_CR register to protect it from accidental writes.
    FLASH->CR |= FLASH_CR_LOCK;
}

// Main function to perform the flash erase
void execute_flash_erase(uint8_t sector_number, uint8_t no_of_sectors)
{
    // Wait for any ongoing flash operation to complete.
    while (FLASH->SR & FLASH_SR_BSY)
    {
        // Do nothing, just wait.
    }

    // Unlock the flash for erase operation.
    flash_unlock();

    // Perform a mass erase if the sector number is 0xFF.
    if (sector_number == 0xFF)
    {
        // Enable Mass Erase.
        FLASH->CR |= FLASH_CR_MER;

        // Set the voltage range.
        FLASH->CR |= FLASH_VOLTAGE_RANGE_3 << 8; // Mapped to the PSIZE bits

        // Start the erase operation.
        FLASH->CR |= FLASH_CR_STRT;

        // Wait for the BSY flag to clear.
        while (FLASH->SR & FLASH_SR_BSY)
        {
            // Wait for erase to finish.
        }

        // Clear the Mass Erase bit.
        FLASH->CR &= ~FLASH_CR_MER;
    }
    else
    {
        // Perform sector erase.
        for (uint8_t i = 0; i < no_of_sectors; i++)
        {
            // Clear all error flags in the Status Register.
            FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |
                         FLASH_SR_WRPERR | FLASH_SR_EOP);

            // Set the SER (Sector Erase) bit.
            FLASH->CR |= FLASH_CR_SER;

            // Select the sector to erase.
            // The sector bits (SNB) are 3 to 6 in the FLASH_CR register.
            FLASH->CR &= ~FLASH_CR_SNB; // Clear previous sector selection
            FLASH->CR |= (sector_number + i) << 3;

            // Set the voltage range.
            FLASH->CR |= FLASH_VOLTAGE_RANGE_3 << 8; // Mapped to the PSIZE bits

            // Start the erase operation.
            FLASH->CR |= FLASH_CR_STRT;

            // Wait for the BSY flag to clear.
            while (FLASH->SR & FLASH_SR_BSY)
            {
                // Wait for erase to finish.
            }

            // Clear the SER bit.
            FLASH->CR &= ~FLASH_CR_SER;
        }
    }

    // Lock the flash after the operation.
    flash_lock();
}

void bootloader_handle_flash_erase(uint8_t *pBuffer)
{
    // The command packet format is:
    // [Total length - 1] [Command code] [Sector Number] [Number of sectors] [CRC (4 bytes)]

    uint32_t command_packet_len = pBuffer[0] + 1;
    uint32_t crc_value = 0;
    uint32_t calculated_crc_status = 0;

    // 1. Verify the CRC of the received command packet
    // The CRC to be verified is located at the end of the packet
    crc_value = *(uint32_t *)(pBuffer + command_packet_len - 4);
    uint32_t data_len = command_packet_len - 4;
    calculated_crc_status = bootloader_verify_crc(pBuffer, data_len, crc_value);

    if (calculated_crc_status == VERIFY_CRC_SUCCESS)
    {
        // Checksum is correct, send an ACK
        bootloader_send_ack(1); // 1 byte to follow (status)

        // 2. Extract the sector number and number of sectors from the buffer
        // The sector number is at index 2, and number of sectors is at index 3
        uint8_t sector_number = pBuffer[1];
        uint8_t no_of_sectors = pBuffer[2];

        // 3. Execute the flash erase operation.
        execute_flash_erase(sector_number, no_of_sectors);

        // 4. Send a success status back to the host.
        uint8_t success_status = 0x01; // Example status for success
        bootloader_uart_write_data(&success_status, 1);
    }
    else
    {
        // Checksum is incorrect, send a NACK
        bootloader_send_nack();
    }
}

static void execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    // Wait for any ongoing flash operation to complete.
    while (FLASH->SR & FLASH_SR_BSY)
    {
        // Do nothing, just wait.
    }

    // Unlock the flash for program operation.
    flash_unlock();

    // Set the PG (Program) bit in the Control Register (CR).
    FLASH->CR |= FLASH_CR_PG;

    // Set the PSIZE (Program Size) bits for byte-by-byte programming (x8).
    // For STM32F4, PSIZE bits (CR[9:8]) are cleared to 00 for x8 programming.
    // Ensure they are reset to 00 if they were set to something else.
    // The default reset value is 00, but explicit clearing is safer.
    FLASH->CR &= ~(FLASH_CR_PSIZE); // Clear PSIZE bits

    // Loop to program each byte.
    for (uint32_t i = 0; i < len; i++)
    {
        // Clear all error flags in the Status Register before each program operation.
        FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |
                     FLASH_SR_WRPERR | FLASH_SR_EOP);

        // Wait until the flash is not busy from a previous operation.
        while (FLASH->SR & FLASH_SR_BSY) {}

        // Write the data to the flash address.
        // The address must be aligned to the program size (byte in this case).
        *(volatile uint8_t*)(mem_address + i) = pBuffer[i];
    }

    // Clear the PG bit after all bytes are programmed.
    FLASH->CR &= ~FLASH_CR_PG;

    // Lock the flash after the operation.
    flash_lock();
}

void bootloader_handle_mem_write(uint8_t *pBuffer)
{
	uint32_t command_packet_len = pBuffer[0] + 1;
    uint32_t crc_value = 0;
    uint32_t calculated_crc = 0;

    // 1. Verify the CRC of the received command packet
    // The CRC to be verified is located at the end of the packet
    crc_value = *(uint32_t *)(pBuffer + command_packet_len - 4);

    // The data length for CRC calculation is the total length minus the 4 CRC bytes
    uint32_t data_len = command_packet_len - 4;

    calculated_crc = bootloader_verify_crc(pBuffer, data_len, crc_value);



    uint32_t mem_address = 0;
    uint8_t payload_len = 0;

    /*uint32_t data_len_for_crc = rcv_len - 4;
    crc_value = *(uint32_t *)(pBuffer + data_len_for_crc);

    calculated_crc_status = bootloader_verify_crc(pBuffer, data_len_for_crc, crc_value);*/
    // Poll until transmit data register is empty

    if (calculated_crc == VERIFY_CRC_SUCCESS)
    {



        bootloader_send_ack(1);

        // FIX: address starts at index 2, not 1
        mem_address = (uint32_t)pBuffer[2] |
                (uint32_t)pBuffer[3] << 8 |
                (uint32_t)pBuffer[4] << 16 |
                (uint32_t)pBuffer[5] << 24;
        // FIX: payload_len is at index 6
        payload_len = pBuffer[6];

        if (verify_address(mem_address) == 1)
        {
            execute_mem_write(pBuffer + 7, mem_address, payload_len);

            uint8_t success_status = 0x00;
            bootloader_uart_write_data(&success_status, 1);
        }
        else
        {
            bootloader_send_nack();
        }
    }
    else
    {
        bootloader_send_nack();
    }
}


