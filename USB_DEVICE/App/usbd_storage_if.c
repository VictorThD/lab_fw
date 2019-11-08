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
#define NAND_BLOCK_SIZE 64
#define NAND_BLOCK_NUMBER 1024

#define NAND_ADDR_CONVERT_TABLE_ENTRY_SIZE sizeof(uint16_t)
#define NAND_ADDR_CONVERT_TABLE_SIZE 65536 // 64Kb
#define NAND_USED_ALL_PAGES \
  (NAND_ADDR_CONVERT_TABLE_SIZE / NAND_ADDR_CONVERT_TABLE_ENTRY_SIZE)
#define NAND_USED_BLOCK_NUMBER (NAND_USED_ALL_PAGES / NAND_BLOCK_SIZE)

#define LOGICAL_PAGE_SIZE 512
#define LOGICAL_PAGES_IN_NAND_PAGE (NAND_PAGE_SIZE / LOGICAL_PAGE_SIZE)

// As we don't have enough memory in RAM to hold
// <LBA>-><NAND block/page> table, we reduce count
// of NAND blocks that we use.

#define BITS_IN_BYTE 8

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

// static uint16_t g_nand_addr_convert_table[NAND_USED_ALL_PAGES];
static uint8_t g_nand_read_buffer[NAND_PAGE_SIZE];
static uint8_t g_nand_write_buffer[NAND_PAGE_SIZE];
static int g_flag;
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

#define BLK_ADDR 64*20
HAL_StatusTypeDef g_state = HAL_OK;

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes over USB FS IP
  * @param  lun:
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  // // HAL_NAND_StateTypeDef state = HAL_NAND_GetState(&hnand1); // Возвращает 1 - HAL_NAND_STATE_READY
  // // uint32_t hwstatus = HAL_NAND_Read_Status(&hnand1); // Возвращает 64 - NAND_READY
  // static char msg[100];
  // // sprintf(msg, "STORAGE_Init_FS st: %d, stat: %ld", state, hwstatus);

  // NAND_AddressTypeDef nandAddr;
  // nandAddr.Plane = 0;
  // nandAddr.Page = 0;
  // for (uint16_t i = 0; i < 10/*STORAGE_BLK_NBR*/; i++)
  // {
  //   nandAddr.Block = i;
  //   sprintf(msg, "Block addr: %d\n", nandAddr.Block);
  //   HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
  //   HAL_StatusTypeDef state = HAL_NAND_Erase_Block(&hnand1, &nandAddr);
  //   sprintf(msg, "Erase status: %d\n", state);
  //   HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
  // }
  // memset(g_nand_used_blocks_table, 0, sizeof(g_nand_used_blocks_table));
  g_flag = 0;
  memset(g_nand_read_buffer, 0, NAND_PAGE_SIZE);
  memset(g_nand_write_buffer, 0, NAND_PAGE_SIZE);
  for (int i = 0; i < 10; i++)
  {
    g_nand_write_buffer[i] = 10 - i;
  }

  uint32_t blk_addr = BLK_ADDR;
  NAND_AddressTypeDef nandAddr;

  // LbaBlockAddrToNandAddr(blk_addr + 5, &hnand1, &nandAddr);
  // sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
  //   nandAddr.Plane, nandAddr.Block, nandAddr.Page);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  // g_state = HAL_NAND_Erase_Block(&hnand1, &nandAddr);
  // sprintf(g_uart_msg, "Erase status: %d\n", g_state);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  for (uint16_t i = 0; i < NAND_BLOCK_SIZE; i++)
  {
    LbaBlockAddrToNandAddr(blk_addr + i, &hnand1, &nandAddr);
    sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
      nandAddr.Plane, nandAddr.Block, nandAddr.Page);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

    // if (i == 0 || (i % 2 == 1))
    // {
    //   g_state = HAL_NAND_Write_Page_8b(&hnand1, &nandAddr, g_nand_write_buffer, 1);
    //   sprintf(g_uart_msg, "Write status: %d\n", g_state);
    //   HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    //   sprintf(g_uart_msg, "Write bytes: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
    //     , g_nand_write_buffer[0], g_nand_write_buffer[1], g_nand_write_buffer[2], g_nand_write_buffer[3]
    //     , g_nand_write_buffer[4], g_nand_write_buffer[5], g_nand_write_buffer[6], g_nand_write_buffer[7]
    //     , g_nand_write_buffer[8], g_nand_write_buffer[9], g_nand_write_buffer[10], g_nand_write_buffer[11]
    //     , g_nand_write_buffer[12], g_nand_write_buffer[13], g_nand_write_buffer[14], g_nand_write_buffer[15]
    //     , g_nand_write_buffer[16], g_nand_write_buffer[17], g_nand_write_buffer[18], g_nand_write_buffer[19]);
    //   HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    // }

    g_state = HAL_NAND_Read_Page_8b(&hnand1, &nandAddr, g_nand_read_buffer, 1);
    sprintf(g_uart_msg, "Read status: %d\n", g_state);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    sprintf(g_uart_msg, "Read bytes after: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
      , g_nand_read_buffer[0], g_nand_read_buffer[1], g_nand_read_buffer[2], g_nand_read_buffer[3]
      , g_nand_read_buffer[4], g_nand_read_buffer[5], g_nand_read_buffer[6], g_nand_read_buffer[7]
      , g_nand_read_buffer[8], g_nand_read_buffer[9], g_nand_read_buffer[10], g_nand_read_buffer[11]
      , g_nand_read_buffer[12], g_nand_read_buffer[13], g_nand_read_buffer[14], g_nand_read_buffer[15]
      , g_nand_read_buffer[16], g_nand_read_buffer[17], g_nand_read_buffer[18], g_nand_read_buffer[19]);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  }

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
  /* USER CODE BEGIN 6 */
  // if (!g_flag)
  if (0)
  {
    blk_addr = BLK_ADDR;

    NAND_AddressTypeDef nandAddr;
    LbaBlockAddrToNandAddr(blk_addr, &hnand1, &nandAddr);
    sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
      nandAddr.Plane, nandAddr.Block, nandAddr.Page);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

    sprintf(g_uart_msg, "Read bytes before: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
      , g_nand_read_buffer[0], g_nand_read_buffer[1], g_nand_read_buffer[2], g_nand_read_buffer[3]
      , g_nand_read_buffer[4], g_nand_read_buffer[5], g_nand_read_buffer[6], g_nand_read_buffer[7]
      , g_nand_read_buffer[8], g_nand_read_buffer[9], g_nand_read_buffer[10], g_nand_read_buffer[11]
      , g_nand_read_buffer[12], g_nand_read_buffer[13], g_nand_read_buffer[14], g_nand_read_buffer[15]
      , g_nand_read_buffer[16], g_nand_read_buffer[17], g_nand_read_buffer[18], g_nand_read_buffer[19]);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    g_state = HAL_NAND_Read_Page_8b(&hnand1, &nandAddr, g_nand_read_buffer, 1);
    sprintf(g_uart_msg, "Read status: %d\n", g_state);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    sprintf(g_uart_msg, "Read bytes after: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
      , g_nand_read_buffer[0], g_nand_read_buffer[1], g_nand_read_buffer[2], g_nand_read_buffer[3]
      , g_nand_read_buffer[4], g_nand_read_buffer[5], g_nand_read_buffer[6], g_nand_read_buffer[7]
      , g_nand_read_buffer[8], g_nand_read_buffer[9], g_nand_read_buffer[10], g_nand_read_buffer[11]
      , g_nand_read_buffer[12], g_nand_read_buffer[13], g_nand_read_buffer[14], g_nand_read_buffer[15]
      , g_nand_read_buffer[16], g_nand_read_buffer[17], g_nand_read_buffer[18], g_nand_read_buffer[19]);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);


    // sprintf(g_uart_msg, "Erasing block. LBA: %ld\n", blk_addr);
    // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

    // state = HAL_NAND_Erase_Block(&hnand1, &nandAddr);
    // sprintf(g_uart_msg, "Erase status: %d\n", state);
    // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

    g_state = HAL_NAND_Write_Page_8b(&hnand1, &nandAddr, g_nand_write_buffer, 1);
    sprintf(g_uart_msg, "Write status: %d\n", g_state);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    sprintf(g_uart_msg, "Write bytes: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
      , g_nand_write_buffer[0], g_nand_write_buffer[1], g_nand_write_buffer[2], g_nand_write_buffer[3]
      , g_nand_write_buffer[4], g_nand_write_buffer[5], g_nand_write_buffer[6], g_nand_write_buffer[7]
      , g_nand_write_buffer[8], g_nand_write_buffer[9], g_nand_write_buffer[10], g_nand_write_buffer[11]
      , g_nand_write_buffer[12], g_nand_write_buffer[13], g_nand_write_buffer[14], g_nand_write_buffer[15]
      , g_nand_write_buffer[16], g_nand_write_buffer[17], g_nand_write_buffer[18], g_nand_write_buffer[19]);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

    sprintf(g_uart_msg, "Read bytes before: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
      , g_nand_read_buffer[0], g_nand_read_buffer[1], g_nand_read_buffer[2], g_nand_read_buffer[3]
      , g_nand_read_buffer[4], g_nand_read_buffer[5], g_nand_read_buffer[6], g_nand_read_buffer[7]
      , g_nand_read_buffer[8], g_nand_read_buffer[9], g_nand_read_buffer[10], g_nand_read_buffer[11]
      , g_nand_read_buffer[12], g_nand_read_buffer[13], g_nand_read_buffer[14], g_nand_read_buffer[15]
      , g_nand_read_buffer[16], g_nand_read_buffer[17], g_nand_read_buffer[18], g_nand_read_buffer[19]);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    g_state = HAL_NAND_Read_Page_8b(&hnand1, &nandAddr, g_nand_read_buffer, 1);
    sprintf(g_uart_msg, "Read status: %d\n", g_state);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
    sprintf(g_uart_msg, "Read bytes after: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
      , g_nand_read_buffer[0], g_nand_read_buffer[1], g_nand_read_buffer[2], g_nand_read_buffer[3]
      , g_nand_read_buffer[4], g_nand_read_buffer[5], g_nand_read_buffer[6], g_nand_read_buffer[7]
      , g_nand_read_buffer[8], g_nand_read_buffer[9], g_nand_read_buffer[10], g_nand_read_buffer[11]
      , g_nand_read_buffer[12], g_nand_read_buffer[13], g_nand_read_buffer[14], g_nand_read_buffer[15]
      , g_nand_read_buffer[16], g_nand_read_buffer[17], g_nand_read_buffer[18], g_nand_read_buffer[19]);
    HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

    g_flag = 1;
  }

  //  sprintf(msg, "Read FS lun: %d, blk_addr: %ld, blk_len=%d\n", lun, blk_addr, blk_len);
  //  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
  // //memcpy(buf, g_storage + blk_addr * STORAGE_BLK_SIZ, blk_len * STORAGE_BLK_SIZ);

  // NAND_AddressTypeDef nandAddr;
  // UsbBlockAddrToNandAddr(blk_addr, &hnand1, &nandAddr);
  // sprintf(msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
  //   nandAddr.Plane, nandAddr.Block, nandAddr.Page);
  // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);

  // // for (uint16_t i = 0; i < blk_len; i++)
  // // {
  //   HAL_StatusTypeDef state = HAL_NAND_Read_Page_8b(&hnand1, &nandAddr, g_nand_buffer, 1);
  //   sprintf(msg, "Read status: %d\n", state);
  //   HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
  //   memcpy(buf, g_nand_buffer, STORAGE_BLK_SIZ);
  // // }

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
  /* USER CODE BEGIN 7 */
  // static char msg[100];
  // sprintf(msg, "Write FS lun: %d, blk_addr: %ld, blk_len=%d\n", lun, blk_addr, blk_len);
  // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
  // //memcpy(g_storage + blk_addr * STORAGE_BLK_SIZ, buf, blk_len * STORAGE_BLK_SIZ);

  // NAND_AddressTypeDef nandAddr;
  // UsbBlockAddrToNandAddr(blk_addr, &hnand1, &nandAddr);
  // sprintf(msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
  //   nandAddr.Plane, nandAddr.Block, nandAddr.Page);
  // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);

  // // for (uint16_t i = 0; i < blk_len; i++)
  // // {
  //   memcpy(g_nand_buffer, buf, STORAGE_BLK_SIZ);
  //   HAL_StatusTypeDef state = HAL_NAND_Write_Page_8b(&hnand1, &nandAddr, g_nand_buffer, 1);
  //   sprintf(msg, "Write status: %d\n", state);
  //   HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
  //   SetNandBlockUsed(blk_addr);
  // // }

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
