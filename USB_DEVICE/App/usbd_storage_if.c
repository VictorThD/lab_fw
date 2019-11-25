/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v1.0_Cube
  * @brief          : Memory management layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"
#include "fsmc.h" // "main.h"->"stm32f2xx_hal_conf.h"->"stm32f2xx_hal_nand.h"
#include "usart.h"
#include <string.h>
#include <stdint.h>

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

#define NAND_PAGE_SIZE 2048
#define NAND_SPARE_AREA_SIZE 64
#define NAND_PAGE_WITH_SPARE_AREA_SIZE (NAND_PAGE_SIZE + NAND_SPARE_AREA_SIZE)
#define NAND_BLOCK_SIZE 64
#define NAND_BLOCK_NUMBER 1024

#define NAND_ADDR_CONVERT_TABLE_SIZE 65536 // 64Kb
#define NAND_ADDR_CONVERT_TABLE_ENTRY_TYPE uint16_t
#define NAND_ADDR_CONVERT_TABLE_ENTRY_SIZE \
  sizeof(NAND_ADDR_CONVERT_TABLE_ENTRY_TYPE)
#define NAND_USED_ALL_PAGES \
  (NAND_ADDR_CONVERT_TABLE_SIZE / NAND_ADDR_CONVERT_TABLE_ENTRY_SIZE)
#define NAND_USED_BLOCK_NUMBER (NAND_USED_ALL_PAGES / NAND_BLOCK_SIZE)
#define NAND_ADDR_CONVERT_TABLE_ENTRY_PAGE_BITS_COUNT 6
#define NAND_ADDR_CONVERT_TABLE_ENTRY_PAGE_BITS_MASK 0x003FU
#define NAND_ADDR_CONVERT_TABLE_ENTRY_BLOCK_BITS_COUNT 10
#define NAND_ADDR_CONVERT_TABLE_ENTRY_BLOCK_BITS_MASK 0xFFC0U

#define LOGICAL_PAGE_SIZE 512
#define LOGICAL_PAGES_IN_NAND_PAGE (NAND_PAGE_SIZE / LOGICAL_PAGE_SIZE)

/* USER CODE END PRIVATE_DEFINES */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR (NAND_USED_BLOCK_NUMBER * LOGICAL_PAGES_IN_NAND_PAGE)
// #define STORAGE_BLK_NBR                  64 * 1024 /* <NAND Block size> * <NAND Block number> */
// Do not work on home windows. В Диспетчере устройств пишет: Сбой запроса дескриптора устройства
// Не работает со значениями больше 512
// #define STORAGE_BLK_NBR                  0x20
// #define STORAGE_BLK_SIZ                  2048 /* <NAND Page size> */
#define STORAGE_BLK_SIZ LOGICAL_PAGE_SIZE

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */
  
  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,	
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
}; 
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

static NAND_ADDR_CONVERT_TABLE_ENTRY_TYPE g_nand_addr_convert_table[NAND_USED_ALL_PAGES];
static uint8_t g_nand_read_write_buffer[NAND_PAGE_WITH_SPARE_AREA_SIZE];
static NAND_AddressTypeDef g_nand_last_used_page_addr;
// static HAL_StatusTypeDef g_nand_op_state = HAL_OK;
// static int g_flag;
static char g_uart_msg[100];

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

// static void SetNandBlockUsed(uint32_t blk_addr)
// {
//   static char msg[100];
//   sprintf(msg, "SetNandBlockUsed: blk_addr: %ld, table before[i]: 0x%X",
//     blk_addr, g_nand_used_blocks_table[blk_addr / BITS_IN_BYTE]);
//   HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);

//   g_nand_used_blocks_table[blk_addr / BITS_IN_BYTE] |= (uint8_t)(1 << (blk_addr % 8));

//   sprintf(msg, ", table after[i]: %0x",
//     g_nand_used_blocks_table[blk_addr / BITS_IN_BYTE]);
//   HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
// }

// static void ClearNandBlockUsed(uint32_t blk_addr)
// {
//   g_nand_used_blocks_table[blk_addr / BITS_IN_BYTE] &= (uint8_t)(~(1 << (blk_addr % 8)));
// }

static void LbaBlockAddrToNandAddr(uint32_t blk_addr,
  NAND_HandleTypeDef *hnand, NAND_AddressTypeDef* nand_addr)
{
  uint32_t pagesInPlane = hnand->Config.PlaneSize * hnand->Config.BlockSize;
  nand_addr->Plane = blk_addr / pagesInPlane;
  blk_addr %= pagesInPlane;
  uint32_t pagesInBlock = hnand->Config.BlockSize;
  nand_addr->Block = blk_addr / pagesInBlock;
  nand_addr->Page = blk_addr % pagesInBlock;
}

static void SetConvertTableNandAddr(uint32_t blk_addr, NAND_AddressTypeDef* nand_addr)
{
  g_nand_addr_convert_table[blk_addr] =
    ((nand_addr->Block << NAND_ADDR_CONVERT_TABLE_ENTRY_PAGE_BITS_COUNT)
      | nand_addr->Page);
  NAND_ADDR_CONVERT_TABLE_ENTRY_TYPE entry =
    g_nand_addr_convert_table[blk_addr];
  sprintf(g_uart_msg, "SetConvertTableNandAddr: blk_addr: %ld, entry: 0x%02X\n",
    blk_addr, entry);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
}

static void GetConvertTableNandAddr(uint32_t blk_addr,
  NAND_HandleTypeDef *hnand, NAND_AddressTypeDef* nand_addr)
{
  NAND_ADDR_CONVERT_TABLE_ENTRY_TYPE entry =
    g_nand_addr_convert_table[blk_addr];
  sprintf(g_uart_msg, "GetConvertTableNandAddr: blk_addr: %ld, entry: 0x%02X\n",
    blk_addr, entry);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  if (entry == 0)
  {
    LbaBlockAddrToNandAddr(blk_addr, hnand, nand_addr);
  }
  else
  {
    nand_addr->Plane = 0;
    nand_addr->Block = ((entry & NAND_ADDR_CONVERT_TABLE_ENTRY_BLOCK_BITS_MASK)
      >> NAND_ADDR_CONVERT_TABLE_ENTRY_PAGE_BITS_COUNT);
    nand_addr->Page = (entry & NAND_ADDR_CONVERT_TABLE_ENTRY_PAGE_BITS_MASK);
  }
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

static void PrintMsg(const char* msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
}

static void PrintBlkAddr(uint32_t blk_addr)
{
  sprintf(g_uart_msg, "blk_addr: %ld\n", blk_addr);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
}

static void PrintNandAddr(NAND_AddressTypeDef *nand_addr)
{
  sprintf(g_uart_msg, "Nand Addr. plane: %d, block: %d, page=%d\n",
    nand_addr->Plane, nand_addr->Block, nand_addr->Page);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
}

// static void PrintNandOpStatus(NAND_AddressTypeDef *nand_addr)
// {
//   sprintf(g_uart_msg, "Nand Addr. plane: %d, block: %d, page=%d\n",
//           nand_addr->Plane, nand_addr->Block, nand_addr->Page);
//   HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
// }

static void PrintNandBuffer(uint8_t *buf)
{
  sprintf(g_uart_msg, "Page: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X"
                      ", spare: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9],
          buf[NAND_PAGE_SIZE - 10], buf[NAND_PAGE_SIZE - 9], buf[NAND_PAGE_SIZE - 8], buf[NAND_PAGE_SIZE - 7], buf[NAND_PAGE_SIZE - 6], buf[NAND_PAGE_SIZE - 5], buf[NAND_PAGE_SIZE - 4], buf[NAND_PAGE_SIZE - 3], buf[NAND_PAGE_SIZE - 2], buf[NAND_PAGE_SIZE - 1],
          buf[NAND_PAGE_SIZE], buf[NAND_PAGE_SIZE + 1], buf[NAND_PAGE_SIZE + 2], buf[NAND_PAGE_SIZE + 3], buf[NAND_PAGE_SIZE + 4], buf[NAND_PAGE_SIZE + 5], buf[NAND_PAGE_SIZE + 6], buf[NAND_PAGE_SIZE + 7], buf[NAND_PAGE_SIZE + 8], buf[NAND_PAGE_SIZE + 9],
          buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 10], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 9], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 8], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 7], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 6], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 5], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 4], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 3], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 2], buf[NAND_PAGE_WITH_SPARE_AREA_SIZE - 1]);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  int bound = NAND_PAGE_WITH_SPARE_AREA_SIZE;
  for (int i = 0; i < bound; i++)
  {
    if (buf[i] != 0x30)  
    {
      int count = 10;
      int lowi = ((i - count >= 0) ? (i - count) : 0);
      int highi = ((i + count < bound) ? (i + count) : (bound - 1));
      sprintf(g_uart_msg, "Found index: %d, ", i);
      HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
      for (int j = lowi; j <= highi; j++)
      {
        sprintf(g_uart_msg, "%02X", buf[j]);
        HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
      }
      PrintMsg("\n");
      break;
    }
  }
}

static void PrintNandBufferSpareArea(uint8_t *buf)
{
  sprintf(g_uart_msg, "Spare: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9],
          buf[NAND_SPARE_AREA_SIZE - 10], buf[NAND_SPARE_AREA_SIZE - 9], buf[NAND_SPARE_AREA_SIZE - 8], buf[NAND_SPARE_AREA_SIZE - 7], buf[NAND_SPARE_AREA_SIZE - 6], buf[NAND_SPARE_AREA_SIZE - 5], buf[NAND_SPARE_AREA_SIZE - 4], buf[NAND_SPARE_AREA_SIZE - 3], buf[NAND_SPARE_AREA_SIZE - 2], buf[NAND_SPARE_AREA_SIZE - 1]);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  int bound = NAND_SPARE_AREA_SIZE;
  for (int i = 0; i < bound; i++)
  {
    if (buf[i] != 0x30)
    {
      int count = 10;
      int lowi = ((i - count >= 0) ? (i - count) : 0);
      int highi = ((i + count < bound) ? (i + count) : (bound - 1));
      sprintf(g_uart_msg, "Found index: %d, ", i);
      HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
      for (int j = lowi; j <= highi; j++)
      {
        sprintf(g_uart_msg, "%02X", buf[j]);
        HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
      }
      PrintMsg("\n");
      break;
    }
  }
}

#define ATTRIBUTE_AREA ((uint32_t)0x08000000U)

static HAL_StatusTypeDef K9F1G08U0E_My_HAL_NAND_Read_Page_8b(
  NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumPageToRead)
{
  uint32_t index;
  uint32_t tickstart;
  uint32_t deviceAddress, numPagesRead = 0U, nandAddress, nbpages = NumPageToRead;
  uint8_t *buff = pBuffer;

  /* Check the NAND controller state */
  if (hnand->State == HAL_NAND_STATE_BUSY)
  {
    return HAL_BUSY;
  }
  else if (hnand->State == HAL_NAND_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hnand);

    /* Update the NAND controller state */
    hnand->State = HAL_NAND_STATE_BUSY;

    /* Identify the device address */
    if (hnand->Init.NandBank == FSMC_NAND_BANK2)
    {
      deviceAddress = NAND_DEVICE1;
    }
    else
    {
      deviceAddress = NAND_DEVICE2;
    }

    /* NAND raw address calculation */
    nandAddress = ARRAY_ADDRESS(pAddress, hnand);

    /* Page(s) read loop */
    while ((nbpages != 0U) && (nandAddress < ((hnand->Config.BlockSize) * (hnand->Config.BlockNbr))))
    {
      /* Send read page command sequence */
      *(__IO uint8_t *)((uint32_t)(deviceAddress | CMD_AREA)) = NAND_CMD_AREA_A;
      __DSB();

      *(__IO uint8_t *)((uint32_t)(deviceAddress | ADDR_AREA)) = 0x00U;
      __DSB();
      *(__IO uint8_t *)((uint32_t)(deviceAddress | ADDR_AREA)) = 0x00U;
      __DSB();
      *(__IO uint8_t *)((uint32_t)(deviceAddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandAddress);
      __DSB();
      *(__IO uint8_t *)((uint32_t)(deviceAddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandAddress);
      __DSB();

      *(__IO uint8_t *)((uint32_t)(deviceAddress | ATTRIBUTE_AREA | CMD_AREA)) = NAND_CMD_AREA_TRUE1;
      __DSB();

      if (hnand->Config.ExtraCommandEnable == ENABLE)
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        /* Read status until NAND is ready */
        while (HAL_NAND_Read_Status(hnand) != NAND_READY)
        {
          if ((HAL_GetTick() - tickstart) > NAND_WRITE_TIMEOUT)
          {
            /* Update the NAND controller state */
            hnand->State = HAL_NAND_STATE_ERROR;

            /* Process unlocked */
            __HAL_UNLOCK(hnand);

            return HAL_TIMEOUT;
          }
        }

        /* Go back to read mode */
        *(__IO uint8_t *)((uint32_t)(deviceAddress | CMD_AREA)) = ((uint8_t)0x00U);
        __DSB();
      }

      /* Get Data into Buffer */
      for (index = 0U; index < hnand->Config.PageSize; index++)
      {
        *buff = *(uint8_t *)deviceAddress;
        buff++;
      }

      /* Increment read pages number */
      numPagesRead++;

      /* Decrement pages to read */
      nbpages--;

      /* Increment the NAND address */
      nandAddress = (uint32_t)(nandAddress + 1U);
    }

    /* Update the NAND controller state */
    hnand->State = HAL_NAND_STATE_READY;

    /* Process unlocked */
    __HAL_UNLOCK(hnand);
  }
  else
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static int8_t ReadOnePage(uint8_t *buf, uint32_t blk_addr)
{
  PrintMsg("Read one page. ");
  PrintBlkAddr(blk_addr);

  uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
  sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
    nand_blk_addr, logical_blk_in_nand_blk);
  PrintMsg(g_uart_msg);

  NAND_AddressTypeDef nand_addr;
  // GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  LbaBlockAddrToNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  PrintNandAddr(&nand_addr);

  memset(g_nand_read_write_buffer, 0, NAND_PAGE_WITH_SPARE_AREA_SIZE);

  HAL_StatusTypeDef state = HAL_NAND_Read_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  // HAL_StatusTypeDef state = K9F1G08U0E_My_HAL_NAND_Read_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Read status: %d\n", state);
  PrintMsg(g_uart_msg);
  if (state != HAL_OK)
  {
    return (USBD_FAIL);
  }

  memcpy(buf, g_nand_read_write_buffer + logical_blk_in_nand_blk * LOGICAL_PAGE_SIZE, LOGICAL_PAGE_SIZE);

  PrintNandBuffer(g_nand_read_write_buffer);
  // sprintf(g_uart_msg, "Read bytes: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], buf[16], buf[17], buf[18], buf[19]);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  memset(g_nand_read_write_buffer, 0, NAND_PAGE_WITH_SPARE_AREA_SIZE);

  state = HAL_NAND_Read_SpareArea_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Read spare area status: %d\n", state);
  PrintMsg(g_uart_msg);
  if (state != HAL_OK)
  {
    return (USBD_FAIL);
  }

  PrintNandBufferSpareArea(g_nand_read_write_buffer);

  return (USBD_OK);
}

static int8_t WriteOnePage(uint8_t *buf, uint32_t blk_addr)
{
  HAL_StatusTypeDef nand_op_state = HAL_OK;

  sprintf(g_uart_msg, "Write one page: blk_addr: %ld\n", blk_addr);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
  sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
          nand_blk_addr, logical_blk_in_nand_blk);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  NAND_AddressTypeDef nand_addr;
  GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
          nand_addr.Plane, nand_addr.Block, nand_addr.Page);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  // Read page
  nand_op_state = HAL_NAND_Read_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Read status: %d\n", nand_op_state);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  if (nand_op_state != HAL_OK)
  {
    return (USBD_FAIL);
  }
  sprintf(g_uart_msg, "Bytes to write: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], buf[16], buf[17], buf[18], buf[19]);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  memcpy(g_nand_read_write_buffer + logical_blk_in_nand_blk * LOGICAL_PAGE_SIZE,
         buf, LOGICAL_PAGE_SIZE);

  // Look for unused page
  NAND_AddressTypeDef new_nand_addr = g_nand_last_used_page_addr;
  if (HAL_NAND_Address_Inc(&hnand1, &new_nand_addr) != NAND_VALID_ADDRESS)
  {
    return (USBD_FAIL);
  }
  sprintf(g_uart_msg, "New nand Addr: plane: %d, block: %d, page=%d\n",
          new_nand_addr.Plane, new_nand_addr.Block, new_nand_addr.Page);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  if (new_nand_addr.Page == 0)
  {
    // Erase if we start to write in new block
    nand_op_state = HAL_NAND_Erase_Block(&hnand1, &new_nand_addr);
    sprintf(g_uart_msg, "Erase status: %d\n", nand_op_state);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    if (nand_op_state != HAL_OK)
    {
      return (USBD_FAIL);
    }
  }

  // Write page
  nand_op_state = HAL_NAND_Write_Page_8b(&hnand1, &new_nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Write status: %d\n", nand_op_state);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  if (nand_op_state != HAL_OK)
  {
    return (USBD_FAIL);
  }
  SetConvertTableNandAddr(nand_blk_addr, &new_nand_addr);
  g_nand_last_used_page_addr = new_nand_addr;

  return (USBD_OK);
}

static int8_t WriteOnePageFull(uint8_t *buf, uint32_t blk_addr)
{
  HAL_StatusTypeDef nand_op_state = HAL_OK;

  PrintMsg("Write one page. ");
  PrintBlkAddr(blk_addr);
  // sprintf(g_uart_msg, "Write one page: blk_addr: %ld\n", blk_addr);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
  sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
          nand_blk_addr, logical_blk_in_nand_blk);
  PrintMsg(g_uart_msg);

  NAND_AddressTypeDef nand_addr;
  GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  PrintNandAddr(&nand_addr);

  // Write page
  nand_op_state = HAL_NAND_Write_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Write status: %d\n", nand_op_state);
  PrintMsg(g_uart_msg);
  if (nand_op_state != HAL_OK)
  {
    return (USBD_FAIL);
  }

  return (USBD_OK);
}

static int8_t EraseOneBlock(uint32_t blk_addr)
{
  HAL_StatusTypeDef nand_op_state = HAL_OK;

  PrintMsg("Erase block. ");
  PrintBlkAddr(blk_addr);

  uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
  sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
          nand_blk_addr, logical_blk_in_nand_blk);
  PrintMsg(g_uart_msg);

  NAND_AddressTypeDef nand_addr;
  // GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  LbaBlockAddrToNandAddr(nand_blk_addr, &hnand1, &nand_addr);

  nand_op_state = HAL_NAND_Erase_Block(&hnand1, &nand_addr);
  sprintf(g_uart_msg, "Erase status: %d\n", nand_op_state);
  PrintMsg(g_uart_msg);
  if (nand_op_state != HAL_OK)
  {
    return (USBD_FAIL);
  }

  uint32_t tickstart;
  /* Get tick */
  tickstart = HAL_GetTick();

  /* Read status until NAND is ready */
  while (HAL_NAND_Read_Status(&hnand1) != NAND_READY)
  {
    if ((HAL_GetTick() - tickstart) > NAND_WRITE_TIMEOUT)
    {
      PrintMsg("Erase Timeout\n");
      return (USBD_FAIL);
    }
  }
  sprintf(g_uart_msg, "Erase Ok. time: %ld\n", HAL_GetTick() - tickstart);
  PrintMsg(g_uart_msg);

  return (USBD_OK);
}

static int g_stage = 0;
static uint8_t g_logical_buffer_r[LOGICAL_PAGE_SIZE] = {11, 12, 13, 14, 15, 20, 21, 22, 23, 24, 25};
static uint8_t g_logical_buffer_w[LOGICAL_PAGE_SIZE] = {11, 12, 13, 14, 15, 20, 21, 22, 23, 24, 25};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes over USB FS IP
  * @param  lun:
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  memset(g_nand_addr_convert_table, 0, NAND_USED_ALL_PAGES * NAND_ADDR_CONVERT_TABLE_ENTRY_SIZE);
  

  // uint32_t blk_addr;
  // blk_addr = 0;

  // EraseOneBlock(0);

  uint32_t count_bytes = 8;
  uint32_t start_number = 30;

  // memset(g_nand_read_write_buffer, 0, NAND_PAGE_WITH_SPARE_AREA_SIZE);
  // for (uint32_t i = 0; i < 3; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)i;
  // for (uint32_t i = NAND_PAGE_SIZE - 3; i < NAND_PAGE_SIZE; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(NAND_PAGE_SIZE - i - 1);
  // for (uint32_t i = NAND_PAGE_SIZE; i < NAND_PAGE_SIZE + 3; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(i + 5);
  // for (uint32_t i = NAND_PAGE_WITH_SPARE_AREA_SIZE - 3; i < NAND_PAGE_WITH_SPARE_AREA_SIZE; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(NAND_PAGE_WITH_SPARE_AREA_SIZE - i - 1 + 5);
  // PrintNandBuffer(g_nand_read_write_buffer);
  // WriteOnePageFull(g_nand_read_write_buffer, 0);

  // memset(g_nand_read_write_buffer, 0, NAND_PAGE_WITH_SPARE_AREA_SIZE);
  // for (uint32_t i = 0; i < count_bytes; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(i + start_number);
  // for (uint32_t i = NAND_PAGE_SIZE - count_bytes; i < NAND_PAGE_SIZE; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(NAND_PAGE_SIZE - i - 1 + start_number);
  // for (uint32_t i = NAND_PAGE_SIZE; i < NAND_PAGE_SIZE + count_bytes; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(i + start_number + 5);
  // for (uint32_t i = NAND_PAGE_WITH_SPARE_AREA_SIZE - count_bytes; i < NAND_PAGE_WITH_SPARE_AREA_SIZE; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(NAND_PAGE_WITH_SPARE_AREA_SIZE - i - 1 + start_number + 5);
  // PrintNandBuffer(g_nand_read_write_buffer);
  // WriteOnePageFull(g_nand_read_write_buffer, 0);

  // memset(g_nand_read_write_buffer, 0, NAND_PAGE_WITH_SPARE_AREA_SIZE);
  // for (uint32_t i = 0; i < 8; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(i + 10);
  // for (uint32_t i = NAND_PAGE_SIZE - 8; i < NAND_PAGE_SIZE; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(NAND_PAGE_SIZE - i - 1 + 10);
  // for (uint32_t i = NAND_PAGE_SIZE; i < NAND_PAGE_SIZE + 8; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(i + 15);
  // for (uint32_t i = NAND_PAGE_WITH_SPARE_AREA_SIZE - 8; i < NAND_PAGE_WITH_SPARE_AREA_SIZE; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(NAND_PAGE_WITH_SPARE_AREA_SIZE - i - 1 + 15);
  // PrintNandBuffer(g_nand_read_write_buffer);
  // WriteOnePageFull(g_nand_read_write_buffer, 0 + LOGICAL_PAGES_IN_NAND_PAGE);

  // memset(g_nand_read_write_buffer, 0, NAND_PAGE_WITH_SPARE_AREA_SIZE);
  // for (uint32_t i = 0; i < count_bytes; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(i + start_number);
  // for (uint32_t i = NAND_PAGE_SIZE - count_bytes; i < NAND_PAGE_SIZE; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(NAND_PAGE_SIZE - i - 1 + start_number);
  // for (uint32_t i = NAND_PAGE_SIZE; i < NAND_PAGE_SIZE + count_bytes; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(i + start_number + 5);
  // for (uint32_t i = NAND_PAGE_WITH_SPARE_AREA_SIZE - count_bytes; i < NAND_PAGE_WITH_SPARE_AREA_SIZE; i++)
  //   g_nand_read_write_buffer[i] = (uint8_t)(NAND_PAGE_WITH_SPARE_AREA_SIZE - i - 1 + start_number + 5);
  // PrintNandBuffer(g_nand_read_write_buffer);
  // WriteOnePageFull(g_nand_read_write_buffer, 0 + 2*LOGICAL_PAGES_IN_NAND_PAGE);

  ReadOnePage(g_logical_buffer_r, 0);
  ReadOnePage(g_logical_buffer_r, 0 + LOGICAL_PAGES_IN_NAND_PAGE);
  ReadOnePage(g_logical_buffer_r, 0 + 2*LOGICAL_PAGES_IN_NAND_PAGE);

  // LbaBlockAddrToNandAddr(NAND_USED_ALL_PAGES - 1, &hnand1, &g_nand_last_used_page_addr);
  // sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
  //   g_nand_last_used_page_addr.Plane, g_nand_last_used_page_addr.Block,
  //   g_nand_last_used_page_addr.Page);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  .
  * @param  lun: .
  * @param  block_num: .
  * @param  block_size: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}




/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  return HAL_OK;

      /* USER CODE BEGIN 6 */
      if (g_stage == 0 || g_stage == 2)
  {
    sprintf(g_uart_msg, "Stage read\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

    ReadOnePage(g_logical_buffer_r, 4); //0x8000 * 
  }
  else if (g_stage == 1)
  {
    sprintf(g_uart_msg, "Stage write\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

    WriteOnePage(g_logical_buffer_w, 4);
  }
  g_stage++;

  return HAL_OK;

  sprintf(g_uart_msg, "Read FS lun: %d, blk_addr: %ld, blk_len=%d\n", lun, blk_addr, blk_len);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
  sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
    nand_blk_addr, logical_blk_in_nand_blk);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  NAND_AddressTypeDef nand_addr;
  GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
    nand_addr.Plane, nand_addr.Block, nand_addr.Page);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  HAL_StatusTypeDef state = HAL_NAND_Read_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Read status: %d\n", state);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  if (state != HAL_OK)
  {
    return (USBD_FAIL);
  }

  memcpy(buf, g_nand_read_write_buffer + logical_blk_in_nand_blk * LOGICAL_PAGE_SIZE, LOGICAL_PAGE_SIZE);
  sprintf(g_uart_msg, "Read bytes: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
    , buf[0], buf[1], buf[2], buf[3]
    , buf[4], buf[5], buf[6], buf[7]
    , buf[8], buf[9], buf[10], buf[11]
    , buf[12], buf[13], buf[14], buf[15]
    , buf[16], buf[17], buf[18], buf[19]);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  return HAL_OK;

  /* USER CODE BEGIN 7 */
  HAL_StatusTypeDef nand_op_state = HAL_OK;

  sprintf(g_uart_msg, "Write FS lun: %d, blk_addr: %ld, blk_len=%d\n", lun, blk_addr, blk_len);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
  sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
    nand_blk_addr, logical_blk_in_nand_blk);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  NAND_AddressTypeDef nand_addr;
  GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
    nand_addr.Plane, nand_addr.Block, nand_addr.Page);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  // Read page
  nand_op_state = HAL_NAND_Read_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Read status: %d\n", nand_op_state);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  if (nand_op_state != HAL_OK)
  {
    return (USBD_FAIL);
  }
  sprintf(g_uart_msg, "Bytes to write: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
    , buf[0], buf[1], buf[2], buf[3]
    , buf[4], buf[5], buf[6], buf[7]
    , buf[8], buf[9], buf[10], buf[11]
    , buf[12], buf[13], buf[14], buf[15]
    , buf[16], buf[17], buf[18], buf[19]);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  memcpy(g_nand_read_write_buffer + logical_blk_in_nand_blk * LOGICAL_PAGE_SIZE,
    buf, LOGICAL_PAGE_SIZE);

  // Look for unused page
  NAND_AddressTypeDef new_nand_addr = g_nand_last_used_page_addr;
  if (HAL_NAND_Address_Inc(&hnand1, &new_nand_addr) != NAND_VALID_ADDRESS)
  {
    return (USBD_FAIL);
  }
  sprintf(g_uart_msg, "New nand Addr: plane: %d, block: %d, page=%d\n",
    new_nand_addr.Plane, new_nand_addr.Block, new_nand_addr.Page);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  if (new_nand_addr.Page == 0)
  {
    // Erase if we start to write in new block
    nand_op_state = HAL_NAND_Erase_Block(&hnand1, &new_nand_addr);
    sprintf(g_uart_msg, "Erase status: %d\n", nand_op_state);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    if (nand_op_state != HAL_OK)
    {
      return (USBD_FAIL);
    }
  }

  // Write page
  nand_op_state = HAL_NAND_Write_Page_8b(&hnand1, &new_nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Write status: %d\n", nand_op_state);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  if (nand_op_state != HAL_OK)
  {
    return (USBD_FAIL);
  }
  SetConvertTableNandAddr(nand_blk_addr, &new_nand_addr);
  g_nand_last_used_page_addr = new_nand_addr;

  return (USBD_OK);
  /* USER CODE END 7 */
}

/**
  * @brief  .
  * @param  None
  * @retval .
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
