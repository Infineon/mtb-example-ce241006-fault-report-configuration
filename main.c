/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for PSOC4 HVMS Fault report example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include <stdio.h>
#include <inttypes.h>

/*******************************************************************************
* Data Structures
********************************************************************************/
/* Number of error injection bits */
typedef enum
{
    INJECT_NONE = 0,
    INJECT_1BIT,
    INJECT_2BIT,
} InjectType;

/*******************************************************************************
* Macros
********************************************************************************/
/* Target SRAM address */
#define RAM_ADDRESS               ((uint32_t)&g_test)

/* Offset of target SRAM from top of SRAM */
#define RAM_OFFSET                (RAM_ADDRESS - CY_SRAM_BASE)

/* Value to be used on error injection */
#define RAM_CORRECT_DATA          (0x5A5A5A5A5A5A5A5Aull)
#define RAM_ONEBIT_DATA           (RAM_CORRECT_DATA ^ 1)
#define RAM_TWOBIT_DATA           (RAM_CORRECT_DATA ^ 3)

/* Parity to be used on error injection */
#define RAM_CORRECT_PARITY(p)     (generate_Parity(make_Word_For_Sram_Ecc(p)))
#define RAM_ONEBIT_PARITY(p)      (RAM_CORRECT_PARITY(p) ^ 1)
#define RAM_TWOBIT_PARITY(p)      (RAM_CORRECT_PARITY(p) ^ 3)

#define LED_BLINK_INTERVAL_MS     (500)
#define SLEEP_INTERVAL_MS         (50)

#define KILO_BYTE_TO_BYTE         (1024)
#define IGNORE_BYTE               (4)
#define SHIFTED_BIT               (2)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Variable for using as the testing target address */
uint64_t g_test;

/* Fault configuration for interrupt */
const cy_stc_sysfault_config_t  fault_cfg_intr =
{
    .triggerEnable = true,
    .outputEnable  = true,
    .resetEnable   = false,
};

/* Fault configuration for reset*/
const cy_stc_sysfault_config_t fault_cfg_reset =
{
    .triggerEnable = true,
    .outputEnable  = true,
    .resetEnable   = true,
};

/* Configure Interrupt for Fault structure */
const cy_stc_sysint_t irq_cfg =
{
    .intrSrc = cpuss_interrupt_fault_0_IRQn,
    .intrPriority = 2UL
};

/*******************************************************************************
* Function Name: handle_Fault_IRQ
********************************************************************************
* Summary:
*  Fault interrupt handler function. Display the information of the fault on
*  terminal and blinking LED.
*
* Parameters:
*  none
*
* Return:
*  none
*
********************************************************************************/
void handle_Fault_IRQ(void)
{
    uint32_t faultAddress, faultInfo;
    cy_en_sysfault_source_t errorSource;

    /* Get Fault data */
    faultAddress = Cy_SysFault_GetFaultData(FAULT_STRUCT0, CY_SYSFAULT_DATA0);
    faultInfo = Cy_SysFault_GetFaultData(FAULT_STRUCT0, CY_SYSFAULT_DATA1);
    errorSource = Cy_SysFault_GetErrorSource(FAULT_STRUCT0);

    /* Display the fault information */
    if(errorSource == CPUSS_FAULT_RAMC_C_ECC)
    {
        printf("CPUSS_FAULT_RAMC_C_ECC fault detected in structure 0:\r\n");
        printf("Address:     0x%08" PRIx32 "\r\n", faultAddress);
        printf("Information: 0x%08" PRIx32 "\r\n\n", faultInfo);
    }
    else if (errorSource == CY_SYSFAULT_NO_FAULT)
    {
        printf("CY_SYSFAULT_NO_FAULT fault detected in structure 0:\r\n");
    }
    else
    {
        printf("Fault detected\r\n");
        printf("GetErrorSource: 0x%08" PRIx32 "\r\n", (uint32_t)errorSource);
    }

    /* Blink LED three times */
    for(uint8_t i = 0 ; i < 6 ; i++)
    {
        Cy_GPIO_Inv(CYBSP_LED6_PORT, CYBSP_LED6_PIN);
        Cy_SysLib_Delay(LED_BLINK_INTERVAL_MS);
    }

    /* Clear Interrupt flag */
    Cy_SysFault_ClearInterrupt(FAULT_STRUCT0);

    /* Disable ECC error Injection */
    CPUSS->ECC_TEST = 0;
    CPUSS->RAM_CTL &= ~(1 << CPUSS_RAM_CTL_ECC_INJ_EN_Pos);
}

/*******************************************************************************
* Function Name: Cy_BootStatus
********************************************************************************
* Summary:
*  This function is redefinition of Default Handler for Boot-Up Status and
*  is called when the boot-up is operating after fault reset occurs.
*  It is redefined to ignore the boot status check violation caused by fault.
*
* Parameters:
*  none
*
* Return:
*  none
*
********************************************************************************/
void Cy_BootStatus(void)
{
    /* Users can redefine the boot status check operation */
}

/*******************************************************************************
* Function Name: init_Fault
********************************************************************************
* Summary:
*  Initialize RAM_ECC faults.
*
* Parameters:
*  config - fault control configuration
*
* Return:
*  none
*
********************************************************************************/
void init_Fault(cy_stc_sysfault_config_t *config)
{
    /* clear status */
    Cy_SysFault_ClearStatus(FAULT_STRUCT0);
    Cy_SysFault_SetMaskByIdx(FAULT_STRUCT0, CPUSS_FAULT_RAMC_C_ECC);
    Cy_SysFault_SetMaskByIdx(FAULT_STRUCT0, CPUSS_FAULT_RAMC_NC_ECC);
    Cy_SysFault_SetInterruptMask(FAULT_STRUCT0);
    if (Cy_SysFault_Init(FAULT_STRUCT0, config) != CY_SYSFAULT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* set up the interrupt */
    Cy_SysInt_Init(&irq_cfg, &handle_Fault_IRQ);
    NVIC_SetPriority((IRQn_Type) cpuss_interrupt_fault_0_IRQn, 2UL);
    NVIC_EnableIRQ((IRQn_Type) cpuss_interrupt_fault_0_IRQn);
}

/*******************************************************************************
* Function Name: make_Word_For_Sram_Ecc
********************************************************************************
* Summary:
*  Generate code word used for parity calculation
*
* Parameters:
*  get address for ECC error injection
*
* Return:
*  uint64_t - code word
*
********************************************************************************/
static uint64_t make_Word_For_Sram_Ecc(uint64_t *addr)
{
    /* CODEWORD_SW
     * CODEWORD_SW[31:0] = ACTUALWORD[31:0];
     * Get the lower 32 bits of the actual word.
     * Other bits = 0                            */
    uint64_t actualWord = *addr & 0xFFFFFFFF;

    /* CODEWORD_SW[ADDR_WIDTH+29:32] = ADDR[ADDR_WIDTH-1:2];
     * Calculate the address offset from the base of SRAM.   */
    uint64_t addrOffset = (((uint32_t)addr - CY_SRAM_BASE) & \
          (CPUSS_SRAM_SIZE * KILO_BYTE_TO_BYTE - IGNORE_BYTE)) >> SHIFTED_BIT;

    uint64_t codeWord = actualWord | (addrOffset << 32);

    return codeWord;
}

/*******************************************************************************
* Function Name: do_Reduction_XOR_64bit
********************************************************************************
* Summary:
*  Do reduction XOR operation on specified 64bit value and get the parity.
*
* Parameters:
*  data - target value
*
* Return:
*  uint8_t - parity
*
********************************************************************************/
static uint8_t do_Reduction_XOR_64bit(uint64_t data)
{
    uint64_t parity = 0;
    uint64_t bit    = 0;
    for(uint64_t iPos = 0; iPos < 64; iPos++)
    {
        bit = (data & (1ull << iPos)) >> iPos;
        parity ^= bit;
    }

    return (uint8_t)parity;
}

/*******************************************************************************
* Function Name: generate_Parity
********************************************************************************
* Summary:
*  Calculate parity for specified 64bit value
*
* Parameters:
*  word - target value
*
* Return:
*  uint8_t - parity
*
********************************************************************************/
static uint8_t generate_Parity(uint64_t word)
{
    static const uint64_t ECC_P[7] =
    {
        0x037F36DB22542AABull,
        0x05BDEB5A44994D35ull,
        0x09DDDCEE08E271C6ull,
        0x11EEBBA98F0381F8ull,
        0x21F6D775F003FE00ull,
        0x41FB6DB4FFFC0000ull,
        0x8103FFF8112C965Full,
    };

    uint8_t ecc = 0;
    for (uint32_t cnt = 0; cnt < (sizeof(ECC_P) / sizeof(ECC_P[0])); cnt++)
    {
        ecc |= (do_Reduction_XOR_64bit(word & ECC_P[cnt]) << cnt);
    }

    return ecc;
}

/*******************************************************************************
* Function Name: inject_ECC_SRAM_ECC_error
********************************************************************************
* Summary:
*  Inject ECC error. Errors with a total of 3 bits or more cannot be set
*  because they cannot be detected correctly.
*
* Parameters:
*  ramAddr - target RAM address (should be point to SRAM region)
*  injectVal - num of error bits to be injected for value
*  injectParity - num of error bits to be injected for parity
*
* Return:
*  uint8_t - parity
*
********************************************************************************/
void inject_ECC_SRAM_ECC_error(uint64_t *ramAddr,\
                               InjectType injectVal,\
                               InjectType injectParity)
{
    uint8_t  parityCorrect, parity1bit, parity2bit;
    uint8_t  *parity;
    volatile static uint64_t ramVal;

    /* first, set correct data to calculate specified parity */
    *ramAddr = RAM_CORRECT_DATA;
    parityCorrect = RAM_CORRECT_PARITY(ramAddr);
    parity1bit = RAM_ONEBIT_PARITY(ramAddr);
    parity2bit = RAM_TWOBIT_PARITY(ramAddr);

    /* calculate specified parity */
    /* correct parity */
    if (injectParity == INJECT_NONE)
    {
        parity = &parityCorrect;
    }
    /* 1bit inverted parity */
    else if (injectParity == INJECT_1BIT)
    {
        /* 2bit inverted value is specified */
        if (injectVal == INJECT_2BIT)
        {
            /* ECC will not work well when more than 2bit are inverted */
            CY_ASSERT(0);
        }
        parity = &parity1bit;
    }
    /* 2bit inverted parity */
    else
    {
        /* incorrect value is specified */
        if (injectVal != INJECT_NONE)
        {
            /* ECC will not work well when more than 2bit are inverted */
            CY_ASSERT(0);
        }
        parity = &parity2bit;
    }

    /* set specified value to target RAM */
    /* correct value */
    if (injectVal == INJECT_NONE)
    {
        *ramAddr = RAM_CORRECT_DATA;
    }
    /* 1bit inverted value */
    else if (injectVal == INJECT_1BIT)
    {
        *ramAddr = RAM_ONEBIT_DATA;
    }
    /* 2bit inverted value */
    else
    {
        *ramAddr = RAM_TWOBIT_DATA;
    }
    /* keep the specified value */
    ramVal = *ramAddr;

    /* Specifying Parity word for RAM address (Bits [31:25])*/
    CPUSS->ECC_TEST |= ((uint32_t)*parity << CPUSS_ECC_TEST_SYND_DATA_Pos);

    /* Specifying RAM address for ECC injection (Bits [24:0]) */
    CPUSS->ECC_TEST = (CPUSS_ECC_TEST_WORD_ADDR_Msk & \
                      (RAM_OFFSET >> SHIFTED_BIT));


    /* Enable RAM CTL ECC Injection*/
    CPUSS->RAM_CTL |= ((1 << CPUSS_RAM_CTL_ECC_INJ_EN_Pos) | \
                       (1 << CPUSS_RAM_CTL_ECC_ENABLE_Pos) | \
                       (1 << CPUSS_RAM_CTL_ECC_AUTO_CORRECT_Pos));

    /* SRAM data write to set specified parity */
    *ramAddr = ramVal;

    Cy_SysLib_Delay(SLEEP_INTERVAL_MS);

    /* Reads SRAM so that checks are performed with the set parity */
    ramVal = *ramAddr;
}

/*******************************************************************************
* Function Name: generate_ECC_SRAM_correctable_error_by_parity
********************************************************************************
* Summary:
*  Inject ECC 1bit parity error
*
* Parameters:
*  none
*
* Return:
*  none
*
********************************************************************************/
void generate_ECC_SRAM_correctable_error_by_parity(void)
{
    inject_ECC_SRAM_ECC_error(&g_test, INJECT_NONE, INJECT_1BIT);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function.
*
* Parameters:
*  none
*
* Return:
*  int
*
********************************************************************************/
int main(void)
{
    /* Variable for storing character read from terminal */
    uint8_t uartReadValue;

    /* Board init */
    if(cybsp_init() != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, NULL);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);
    cy_retarget_io_init(CYBSP_UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "Fault report configuration example "
           "****************** \r\n\n");

    /* Check if CY_SYSFAULT_RAMC0_C_ECC reset was caused by FAULT_STRUCT0 */
    if((FAULT_STRUCT0->STATUS == CPUSS_FAULT_RAMC_C_ECC) && \
       (CY_SYSLIB_RESET_ACT_FAULT == Cy_SysLib_GetResetReason()))
    {
        printf("Reset caused by FAULT_STRUCT0.\r\n");
        printf("Detected fault: CY_SYSFAULT_RAMC0_C_ECC\r\n");
        printf("Address:     0x%08" PRIx32 "\r\n", \
               Cy_SysFault_GetFaultData(FAULT_STRUCT0, CY_SYSFAULT_DATA0));
        printf("Information: 0x%08" PRIx32 "\r\n\n", \
               Cy_SysFault_GetFaultData(FAULT_STRUCT0, CY_SYSFAULT_DATA1));
    }

    Cy_SysLib_ClearResetReason();

    /* Check for SRAM area validation */
    if(RAM_OFFSET >= CPUSS_SRAM_SIZE * KILO_BYTE_TO_BYTE)
    {
        printf("SRAM for test is not located within SRAM region...\r\n");
        CY_ASSERT(0);
    }

    printf("Press 'i' to generate a SRAM correctable ECC error interrupt\r\n");
    printf("Press 'r' to generate a SRAM correctable ECC error reset\r\n\n");

    for (;;)
    {
        /* Check if 'i' key or 'r' key was pressed */
        uartReadValue = Cy_SCB_UART_Get(CYBSP_UART_HW);

        if(uartReadValue != 0xFF)
        {
            if(uartReadValue == 'i')
            {
                printf("Fault interrupt requested\r\n");

                /* Enable interrupt for fault handling */
                init_Fault((cy_stc_sysfault_config_t *)&fault_cfg_intr);

                /* generate an SRAM correctable ECC error */
                generate_ECC_SRAM_correctable_error_by_parity();
            }
            else if(uartReadValue == 'r')
            {
                printf("Fault reset requested\r\n");

                /* Enable reset for Fault handling */
                init_Fault((cy_stc_sysfault_config_t *)&fault_cfg_reset);

                /* generate an SRAM correctable ECC error */
                generate_ECC_SRAM_correctable_error_by_parity();
            }
        }

        Cy_SysLib_Delay(SLEEP_INTERVAL_MS);
    }
}

/* [] END OF FILE */
