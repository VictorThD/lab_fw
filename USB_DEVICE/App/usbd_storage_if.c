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

#define BITS_IN_BYTE 8

#if ((NAND_BLOCK_NUMBER % BITS_IN_BYTE) == 0)
# define NAND_INVALID_BLOCK_TABLE_SIZE (NAND_BLOCK_NUMBER / BITS_IN_BYTE)
#else
# define NAND_INVALID_BLOCK_TABLE_SIZE ((NAND_BLOCK_NUMBER / BITS_IN_BYTE) + 1)
#endif

#define NAND_USED_BLOCK_NUMBER (NAND_BLOCK_NUMBER / 2)
// Number of pages in all used blocks
#define NAND_USED_PAGE_NUMBER (NAND_USED_BLOCK_NUMBER * NAND_BLOCK_SIZE)
// Type that wide enough to include number of pages in all used blocks
#define NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE uint16_t
#define NAND_ADDR_TRANSLATION_TABLE_ENTRY_SIZE (sizeof(uint16_t))
#define NAND_ADDR_TRANSLATION_TABLE_SIZE                                                      \
  ((((NAND_USED_PAGE_NUMBER * NAND_ADDR_TRANSLATION_TABLE_ENTRY_SIZE) % NAND_PAGE_SIZE) == 0) \
    ? (NAND_USED_PAGE_NUMBER * NAND_ADDR_TRANSLATION_TABLE_ENTRY_SIZE)                        \
    : ((((NAND_USED_PAGE_NUMBER * NAND_ADDR_TRANSLATION_TABLE_ENTRY_SIZE)                      \
      / NAND_PAGE_SIZE) + 1) * NAND_PAGE_SIZE))
#define NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT \
  (NAND_ADDR_TRANSLATION_TABLE_SIZE / NAND_ADDR_TRANSLATION_TABLE_ENTRY_SIZE)
#define NAND_ADDR_TRANSLATION_TABLE_ENTRY_PAGE_BITS_COUNT 6
#define NAND_ADDR_TRANSLATION_TABLE_ENTRY_PAGE_BITS_MASK 0x003FU
#define NAND_ADDR_TRANSLATION_TABLE_ENTRY_BLOCK_BITS_COUNT 10
#define NAND_ADDR_TRANSLATION_TABLE_ENTRY_BLOCK_BITS_MASK 0xFFC0U

// Initial index for the block
// where we will hold address translation table.
// Use the first free block after user blocks
#define NAND_ADDR_TRANSLATION_TABLE_BLOCK_INDEX NAND_USED_BLOCK_NUMBER

// Initial index for the block
// where we will hold our inner configuration data of the NAND.
// Use the last block
#define NAND_INNER_CONFIGURATION_BLOCK_INDEX (NAND_BLOCK_NUMBER - 1)

#define NAND_OP_TIMEOUT 10000 // 10 sec

#define LOGICAL_PAGE_SIZE 512
#define LOGICAL_PAGES_IN_NAND_PAGE (NAND_PAGE_SIZE / LOGICAL_PAGE_SIZE)

#define ATTRIBUTE_AREA ((uint32_t)0x08000000U)

/* USER CODE END PRIVATE_DEFINES */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR (NAND_USED_PAGE_NUMBER * LOGICAL_PAGES_IN_NAND_PAGE)
// #define STORAGE_BLK_NBR                  64 * 1024 /* <NAND Block size> * <NAND Block number> */
// Do not work on home windows. В Диспетчере устройств пишет: Сбой запроса дескриптора устройства
// Не работает со значениями больше 512
// #define STORAGE_BLK_NBR                  0x20
// #define STORAGE_BLK_SIZ                  2048 /* <NAND Page size> */
#define STORAGE_BLK_SIZ LOGICAL_PAGE_SIZE

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
    const int8_t STORAGE_Inquirydata_FS[] = {
    /* 36 */

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
    '0', '.', '0', '1' /* Version      : 4 Bytes */
}; 
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

static uint8_t g_nand_invalid_block_table[NAND_INVALID_BLOCK_TABLE_SIZE];
static NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE
  g_nand_addr_translation_table[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT];
static uint8_t g_nand_read_write_buffer[NAND_PAGE_WITH_SPARE_AREA_SIZE];
// static NAND_AddressTypeDef g_nand_last_used_page_addr;
// static NAND_AddressTypeDef g_nand_free_page_addr;
// static HAL_StatusTypeDef g_nand_op_state = HAL_OK;
// static int g_flag;
typedef struct NandMetadata
{
  NAND_HandleTypeDef *hnand;
  NAND_AddressTypeDef inner_config_addr;
  uint8_t *ibt;
  size_t ibt_size;
  NAND_AddressTypeDef free_page_addr;
  NAND_AddressTypeDef first_non_user_page_addr;
  NAND_AddressTypeDef att_addr;
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE *att;
  size_t att_size;
} NandMetadata;
static NandMetadata g_nand_metadata;

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

static HAL_StatusTypeDef K9F1G08U0E_Format(NAND_HandleTypeDef *hnand,
  NandMetadata *nand_metadata);

static HAL_StatusTypeDef StoreNandMetadata(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata);
static HAL_StatusTypeDef LoadNandMetadata(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata);

static HAL_StatusTypeDef StoreNandInnerConfiguration(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata);
static HAL_StatusTypeDef LoadNandInnerConfiguration(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata);

static HAL_StatusTypeDef StoreAddrTranslationTable(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata);
static HAL_StatusTypeDef LoadAddrTranslationTable(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata);

static HAL_StatusTypeDef EraseUserBlocks(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata);

static int SerializeNandInnerConfiguration(
  const NandMetadata *nand_metadata, uint8_t *buf, size_t buf_size);
static int DeserializeNandInnerConfiguration(
  const uint8_t* buf, size_t buf_size, NandMetadata* nand_metadata);

static HAL_StatusTypeDef K9F1G08U0E_Erase_Block(
    NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress);

// Invalid block table
static int IsNandBlockInvalid(const uint8_t *ibt, size_t ibt_size,
  const NAND_AddressTypeDef *nand_addr);
static void SetNandBlockInvalid(uint8_t *ibt, size_t ibt_size,
  const NAND_AddressTypeDef *nand_addr);

// Address translation table
static int SetAddrTranslationTableNandAddr(
  NandMetadata *nand_metadata,
  uint32_t lba, const NAND_AddressTypeDef *nand_addr);
static int GetAddrTranslationTableNandAddr(
  const NandMetadata *nand_metadata,
  uint32_t lba, NAND_AddressTypeDef* nand_addr);

static void GetAddrTranslationTableEntryNandAddr(
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE att_entry,
  NAND_AddressTypeDef* nand_addr);
static NAND_AddressTypeDef AddrTranslationTableEntryToNandAddr(
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE att_entry);

static void GetNandAddrAddrTranslationTableEntry(
  const NAND_AddressTypeDef* nand_addr,
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE* att_entry);
static NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE NandAddrToAddrTranslationTableEntry(
  const NAND_AddressTypeDef *nand_addr);

// LBA to NAND address conversion

static void LbaToNandAddr(NAND_HandleTypeDef *hnand,
  uint32_t lba, NAND_AddressTypeDef *nand_addr);

// Log

static void PrintMsg(const char* msg);
// static void PrintLba(uint32_t lba);
static void PrintNandAddr(const NAND_AddressTypeDef *nand_addr);
// static void PrintNandOpStatus(NAND_AddressTypeDef *nand_addr);
static void PrintNandBuffer(const uint8_t *buf);
static void PrintNandBufferSpareArea(const uint8_t *buf);
static void PrintBuffer(const uint8_t *buf, size_t buf_size);

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

static HAL_StatusTypeDef ReadOnePage(NandMetadata *nand_metadata,
  uint32_t storage_lba, uint8_t *buf)
{
  HAL_StatusTypeDef status = HAL_OK;

  // PrintMsg("Read one page. ");

  uint32_t lba = storage_lba / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t storage_lba_in_lba = storage_lba % LOGICAL_PAGES_IN_NAND_PAGE;
  // sprintf(g_uart_msg, "storage_lba: %ld, lba: %ld, storage_lba_in_lba: %ld\n",
  //   storage_lba, lba, storage_lba_in_lba);
  // PrintMsg(g_uart_msg);

  NAND_AddressTypeDef nand_addr;
  if (GetAddrTranslationTableNandAddr(nand_metadata, lba, &nand_addr) != 0)
  {
    return HAL_ERROR;
  }
  // LbaToNandAddr(nand_metadata->hnand, lba, &nand_addr);
  // PrintNandAddr(&nand_addr);
  // PrintMsg("\n");

  // memset(g_nand_read_write_buffer, 0, NAND_PAGE_SIZE);

  if ((status = HAL_NAND_Read_Page_8b(nand_metadata->hnand, &nand_addr,
    g_nand_read_write_buffer, 1)) != HAL_OK)
  {
    sprintf(g_uart_msg, "HAL_NAND_Read_Page_8b() failed. Status: %d\n", status);
    PrintMsg(g_uart_msg);

    return status;
  }
  // HAL_StatusTypeDef state = K9F1G08U0E_My_HAL_NAND_Read_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);

  memcpy(buf, g_nand_read_write_buffer + storage_lba_in_lba * LOGICAL_PAGE_SIZE, LOGICAL_PAGE_SIZE);

  // sprintf(g_uart_msg, "Read bytes: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], buf[16], buf[17], buf[18], buf[19]);
  // PrintMsg(g_uart_msg);

  // memset(g_nand_read_write_buffer, 0, NAND_PAGE_WITH_SPARE_AREA_SIZE);

  // state = HAL_NAND_Read_SpareArea_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  // sprintf(g_uart_msg, "Read spare area status: %d\n", state);
  // PrintMsg(g_uart_msg);
  // if (state != HAL_OK)
  // {
  //   return (USBD_FAIL);
  // }

  // PrintNandBufferSpareArea(g_nand_read_write_buffer);

  return HAL_OK;
}

// static int g_fl = 0;

static HAL_StatusTypeDef WriteOnePage(NandMetadata *nand_metadata,
  uint32_t storage_lba, const uint8_t *buf)
{
  HAL_StatusTypeDef status = HAL_OK;

      // if (g_fl)
      // {
      //   return HAL_OK;
      // }
      // else
      // {
      //   g_fl = 1;
      // }

  PrintMsg("Write one page. ");

  if (ARRAY_ADDRESS(&nand_metadata->free_page_addr, nand_metadata->hnand) >=
      ARRAY_ADDRESS(&nand_metadata->first_non_user_page_addr, nand_metadata->hnand))
  {
    PrintMsg("WriteOnePage() failed. Free pages are run out\n");

    return HAL_ERROR;
  }

  uint32_t lba = storage_lba / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t storage_lba_in_lba = storage_lba % LOGICAL_PAGES_IN_NAND_PAGE;
  sprintf(g_uart_msg, "storage_lba: %ld, lba: %ld, storage_lba_in_lba: %ld\n",
    storage_lba, lba, storage_lba_in_lba);
  PrintMsg(g_uart_msg);

  NAND_AddressTypeDef nand_addr;
  if (GetAddrTranslationTableNandAddr(nand_metadata, lba, &nand_addr) != 0)
  {
    return HAL_ERROR;
  }
  // LbaBlockAddrToNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  // PrintNandAddr(&nand_addr);
  // PrintMsg("\n");

  // Read page
  if ((status = HAL_NAND_Read_Page_8b(nand_metadata->hnand, &nand_addr,
    g_nand_read_write_buffer, 1)) != HAL_OK)
  {
    sprintf(g_uart_msg, "HAL_NAND_Read_Page_8b() failed. Status: %d\n", status);
    PrintMsg(g_uart_msg);

    return status;
  }

  sprintf(g_uart_msg, "Bytes to write: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], buf[16], buf[17], buf[18], buf[19]);
  PrintMsg(g_uart_msg);
  memcpy(g_nand_read_write_buffer + storage_lba_in_lba * LOGICAL_PAGE_SIZE,
    buf, LOGICAL_PAGE_SIZE);

  NAND_AddressTypeDef new_nand_addr = nand_metadata->free_page_addr;
  PrintMsg("New ");
  PrintNandAddr(&new_nand_addr);
  PrintMsg("\n");

  // Look for next free page
  if (HAL_NAND_Address_Inc(nand_metadata->hnand,
      &nand_metadata->free_page_addr) != NAND_VALID_ADDRESS)
  {
    PrintMsg("HAL_NAND_Address_Inc() failed: not NAND_VALID_ADDRESS\n");

    return HAL_ERROR;
  }

  // // Erase the block when we go to the next block
  // if (new_nand_addr.Page == 0)
  // {
  //   // Erase if we start to write in new block
  //   if ((status = HAL_NAND_Erase_Block(nand_metadata->hnand, &new_nand_addr)) != HAL_OK)
  //   {
  //     sprintf(g_uart_msg, "HAL_NAND_Erase_Block() failed. Status: %d\n", status);
  //     PrintMsg(g_uart_msg);

  //     return status;
  //   }
  // }

  // Write page
  if ((status = HAL_NAND_Write_Page_8b(nand_metadata->hnand, &new_nand_addr,
      g_nand_read_write_buffer, 1)) != HAL_OK)
  {
    sprintf(g_uart_msg, "HAL_NAND_Write_Page_8b() failed. Status: %d\n", status);
    PrintMsg(g_uart_msg);

    return status;
  }

  // Set new page address in address translation table
  if (SetAddrTranslationTableNandAddr(nand_metadata, lba, &new_nand_addr) != 0)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

// static int8_t WriteOnePageFull(uint8_t *buf, uint32_t blk_addr)
// {
//   HAL_StatusTypeDef nand_op_state = HAL_OK;

//   // PrintMsg("Write one page. ");
//   // PrintBlkAddr(blk_addr);
//   // // sprintf(g_uart_msg, "Write one page: blk_addr: %ld\n", blk_addr);
//   // // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

//   uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
//   // uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
//   // sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
//   //         nand_blk_addr, logical_blk_in_nand_blk);
//   // PrintMsg(g_uart_msg);

//   NAND_AddressTypeDef nand_addr;
//   // GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
//   LbaBlockAddrToNandAddr(nand_blk_addr, &hnand1, &nand_addr);
//   // PrintNandAddr(&nand_addr);

//   // Write page
//   nand_op_state = HAL_NAND_Write_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
//   // sprintf(g_uart_msg, "Write status: %d\n", nand_op_state);
//   // PrintMsg(g_uart_msg);
//   if (nand_op_state != HAL_OK)
//   {
//     return (USBD_FAIL);
//   }

//   return (USBD_OK);
// }

// static int8_t EraseOneBlock(uint32_t blk_addr)
// {
//   HAL_StatusTypeDef nand_op_state = HAL_OK;

//   // PrintMsg("Erase block. ");
//   // PrintBlkAddr(blk_addr);

//   uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
//   // uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
//   // sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
//   //         nand_blk_addr, logical_blk_in_nand_blk);
//   // PrintMsg(g_uart_msg);

//   NAND_AddressTypeDef nand_addr;
//   // GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
//   LbaBlockAddrToNandAddr(nand_blk_addr, &hnand1, &nand_addr);

//   nand_op_state = HAL_NAND_Erase_Block(&hnand1, &nand_addr);
//   sprintf(g_uart_msg, "Erase status: %d\n", nand_op_state);
//   PrintMsg(g_uart_msg);
//   if (nand_op_state != HAL_OK)
//   {
//     return (USBD_FAIL);
//   }

//   uint32_t tickstart;
//   /* Get tick */
//   tickstart = HAL_GetTick();

//   /* Read status until NAND is ready */
//   uint32_t status;
//   while ((status = HAL_NAND_Read_Status(&hnand1)) != NAND_READY)
//   {
//     if ((HAL_GetTick() - tickstart) > NAND_WRITE_TIMEOUT)
//     {
//       PrintMsg("Erase Timeout\n");
//       return (USBD_FAIL);
//     }
//   }
//   sprintf(g_uart_msg, "Erase Ok. time: %ld, status: 0x%lX\n", HAL_GetTick() - tickstart, status);
//   PrintMsg(g_uart_msg);

//   return (USBD_OK);
// }

// static int g_stage = 0;
// static uint8_t g_logical_buffer_r[LOGICAL_PAGE_SIZE] = {11, 12, 13, 14, 15, 20, 21, 22, 23, 24, 25};
// static uint8_t g_logical_buffer_w[LOGICAL_PAGE_SIZE] = {11, 12, 13, 14, 15, 20, 21, 22, 23, 24, 25};

// #define K9F1G08U0E_NAND_FORMAT 1

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes over USB FS IP
  * @param  lun:
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  NAND_AddressTypeDef nand_addr = {0};
  // Initialize with default values.
  // Some values can be changed during loading metadata further
  g_nand_metadata.hnand = &hnand1;
  g_nand_metadata.free_page_addr = nand_addr;
  nand_addr.Block = NAND_ADDR_TRANSLATION_TABLE_BLOCK_INDEX;
  g_nand_metadata.first_non_user_page_addr = nand_addr;
  g_nand_metadata.att_addr = nand_addr;
  nand_addr.Block = NAND_INNER_CONFIGURATION_BLOCK_INDEX;
  g_nand_metadata.inner_config_addr = nand_addr;
  g_nand_metadata.ibt = g_nand_invalid_block_table;
  g_nand_metadata.ibt_size = sizeof(g_nand_invalid_block_table);
  g_nand_metadata.att = g_nand_addr_translation_table;
  g_nand_metadata.att_size = sizeof(g_nand_addr_translation_table);

#ifdef K9F1G08U0E_NAND_FORMAT
  HAL_StatusTypeDef status = HAL_OK;
  if ((status = K9F1G08U0E_Format(&hnand1, &g_nand_metadata)) != HAL_OK)
  {
    sprintf(g_uart_msg, "Format failed. HAL status: %d\n", status);
    PrintMsg(g_uart_msg);
  }
#else

  // memset(g_nand_addr_convert_table, 0, NAND_USED_PAGE_NUMBER * NAND_ADDR_CONVERT_TABLE_ENTRY_SIZE);

  // uint32_t blk_addr;
  // blk_addr = 0;

  // EraseOneBlock(0);

  // uint32_t count_bytes = 8;
  // uint32_t start_number = 50;

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

  // for (int i = 1)
  // EraseOneBlock(0 * NAND_BLOCK_SIZE * LOGICAL_PAGES_IN_NAND_PAGE);
  
  // EraseOneBlock(1 * NAND_BLOCK_SIZE * LOGICAL_PAGES_IN_NAND_PAGE);

  // WriteOnePageFull(g_nand_read_write_buffer, 1 * NAND_BLOCK_SIZE * LOGICAL_PAGES_IN_NAND_PAGE);

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

  // ReadOnePage(g_logical_buffer_r, 0 * NAND_BLOCK_SIZE * LOGICAL_PAGES_IN_NAND_PAGE);
  // ReadOnePage(g_logical_buffer_r, 511 * NAND_BLOCK_SIZE * LOGICAL_PAGES_IN_NAND_PAGE);
  // ReadOnePage(g_logical_buffer_r, 512 * NAND_BLOCK_SIZE * LOGICAL_PAGES_IN_NAND_PAGE);
  // ReadOnePage(g_logical_buffer_r, 1023 * NAND_BLOCK_SIZE * LOGICAL_PAGES_IN_NAND_PAGE);
  // ReadOnePage(g_logical_buffer_r, 0 + LOGICAL_PAGES_IN_NAND_PAGE);
  // ReadOnePage(g_logical_buffer_r, 0 + 2*LOGICAL_PAGES_IN_NAND_PAGE);

  // uint32_t nand_status = HAL_NAND_Read_Status(&hnand1);
  // sprintf(g_uart_msg, "nand_status: 0x%08lX\n", nand_status);
  // PrintMsg(g_uart_msg);

  // LbaBlockAddrToNandAddr(NAND_USED_ALL_PAGES - 1, &hnand1, &g_nand_last_used_page_addr);
  // sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
  //   g_nand_last_used_page_addr.Plane, g_nand_last_used_page_addr.Block,
  //   g_nand_last_used_page_addr.Page);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  // NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE* t = g_nand_metadata.att;
  // for (size_t i = 0; i < 5; i++)
  //   t[i] = (NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE)(i + 10);
  // for (size_t i = NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 6;
  //   i < NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT; i++)
  //   t[i] = (NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE)(NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - i - 1 + 10);

  // sprintf(g_uart_msg, "Table before: %04X%04X%04X%04X%04X%04X%04X%04X%04X%04X %04X%04X%04X%04X%04X%04X%04X%04X%04X%04X\n",
  //         t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8], t[9],
  //         t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 10], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 9], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 8], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 7], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 6],
  //         t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 5], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 4], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 3], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 2], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 1]);
  // PrintMsg(g_uart_msg);

  HAL_StatusTypeDef status = HAL_OK;
  if ((status = LoadNandMetadata(&hnand1, &g_nand_metadata)) != HAL_OK)
  {
    sprintf(g_uart_msg, "LoadNandInnerConfiguration() failed. HAL status: %d\n", status);
    PrintMsg(g_uart_msg);
  }
  PrintMsg("free_page_addr ");
  PrintNandAddr(&g_nand_metadata.free_page_addr);
  PrintMsg("\n");
  PrintMsg("first_non_user_page_addr ");
  PrintNandAddr(&g_nand_metadata.first_non_user_page_addr);
  PrintMsg("\n");
  PrintMsg("att_addr ");
  PrintNandAddr(&g_nand_metadata.att_addr);
  PrintMsg("\n");
  PrintMsg("inner config addr ");
  PrintNandAddr(&g_nand_metadata.inner_config_addr);
  PrintMsg("\n");
  // sprintf(g_uart_msg, "Table after: %04X%04X%04X%04X%04X%04X%04X%04X%04X%04X %04X%04X%04X%04X%04X%04X%04X%04X%04X%04X\n",
  //       t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8], t[9],
  //       t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 10], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 9], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 8], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 7], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 6],
  //       t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 5], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 4], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 3], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 2], t[NAND_ADDR_TRANSLATION_TABLE_ENTRY_COUNT - 1]);
  // PrintMsg(g_uart_msg);

  // uint8_t buf[512];
  // ReadOnePage(&g_nand_metadata, 0, buf);
  // ReadOnePage(&g_nand_metadata, 0 * 64 * LOGICAL_PAGES_IN_NAND_PAGE, buf);
  // ReadOnePage(&g_nand_metadata, 1 * 64 * LOGICAL_PAGES_IN_NAND_PAGE, buf);
  // ReadOnePage(&g_nand_metadata, 2 * 64 * LOGICAL_PAGES_IN_NAND_PAGE, buf);
  // ReadOnePage(&g_nand_metadata, 1023 * 64 * LOGICAL_PAGES_IN_NAND_PAGE, buf);

#endif

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

  // if (g_stage == 0 || g_stage == 2)
  // {
  //   sprintf(g_uart_msg, "Stage read\n");
  //   HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  //   ReadOnePage(g_logical_buffer_r, 4); //0x8000 * 
  // }
  // else if (g_stage == 1)
  // {
  //   sprintf(g_uart_msg, "Stage write\n");
  //   HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  //   WriteOnePage(g_logical_buffer_w, 4);
  // }
  // g_stage++;

  // sprintf(g_uart_msg, "Read FS lun: %d, blk_addr: %ld, blk_len=%d\n", lun, blk_addr, blk_len);
  // PrintMsg(g_uart_msg);

  for (size_t i = 0; i < blk_len; i++)
  {
    ReadOnePage(&g_nand_metadata, blk_addr + i, buf + i * LOGICAL_PAGE_SIZE);
  }
  // uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
  // uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
  // sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
  //   nand_blk_addr, logical_blk_in_nand_blk);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  // NAND_AddressTypeDef nand_addr;
  // // GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
  // sprintf(g_uart_msg, "Nand Addr: plane: %d, block: %d, page=%d\n",
  //   nand_addr.Plane, nand_addr.Block, nand_addr.Page);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  // HAL_StatusTypeDef state = HAL_NAND_Read_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  // sprintf(g_uart_msg, "Read status: %d\n", state);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
  // if (state != HAL_OK)
  // {
  //   return (USBD_FAIL);
  // }

  // memcpy(buf, g_nand_read_write_buffer + logical_blk_in_nand_blk * LOGICAL_PAGE_SIZE, LOGICAL_PAGE_SIZE);
  // sprintf(g_uart_msg, "Read bytes: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n"
  //   , buf[0], buf[1], buf[2], buf[3]
  //   , buf[4], buf[5], buf[6], buf[7]
  //   , buf[8], buf[9], buf[10], buf[11]
  //   , buf[12], buf[13], buf[14], buf[15]
  //   , buf[16], buf[17], buf[18], buf[19]);
  // HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

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

  // sprintf(g_uart_msg, "Write FS lun: %d, blk_addr: %ld, blk_len=%d\n", lun, blk_addr, blk_len);
  // PrintMsg(g_uart_msg);

  for (size_t i = 0; i < blk_len; i++)
  {
    WriteOnePage(&g_nand_metadata, blk_addr, buf + i * LOGICAL_PAGE_SIZE);
  }

  /*
  HAL_StatusTypeDef nand_op_state = HAL_OK;

  sprintf(g_uart_msg, "Write FS lun: %d, blk_addr: %ld, blk_len=%d\n", lun, blk_addr, blk_len);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
  uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
  sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
    nand_blk_addr, logical_blk_in_nand_blk);
  HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);

  NAND_AddressTypeDef nand_addr;
  // GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
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
  // SetConvertTableNandAddr(nand_blk_addr, &new_nand_addr);
  g_nand_last_used_page_addr = new_nand_addr;
*/
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


HAL_StatusTypeDef NandStoreMetadata()
{
  return StoreNandMetadata(g_nand_metadata.hnand, &g_nand_metadata);
}

HAL_StatusTypeDef NandPrintPageData(NAND_AddressTypeDef *nand_addr,
  size_t offset, size_t count)
{
  size_t page_size = NAND_PAGE_SIZE;

  // assert
  if (offset >= page_size - 1)
  {
    sprintf(g_uart_msg, "NandPrintPageData() error. No data to print."
      " offset >= (page size - 1), offset: %d, page size: %d\n", offset, page_size);
    PrintMsg(g_uart_msg);

    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_OK;

  if ((status = HAL_NAND_Read_Page_8b(g_nand_metadata.hnand, nand_addr,
      g_nand_read_write_buffer, 1)) != HAL_OK)
  {
    sprintf(g_uart_msg, "NandPrintPageData() error."
      " HAL_NAND_Read_Page_8b status: %d\n", status);
    PrintMsg(g_uart_msg);

    return status;
  }
  // HAL_StatusTypeDef state = K9F1G08U0E_My_HAL_NAND_Read_Page_8b(&hnand1, &nand_addr, g_nand_read_write_buffer, 1);
  sprintf(g_uart_msg, "Page data: ");
  PrintMsg(g_uart_msg);
  PrintNandAddr(nand_addr);
  size_t print_count = ((offset + count) <= page_size)
    ? count : (page_size - offset);
  sprintf(g_uart_msg, ", offset: %d, count: %d, print_count: %d, data: ",
    offset, count, print_count);
  PrintMsg(g_uart_msg);
  PrintBuffer(&g_nand_read_write_buffer[offset], print_count);
  PrintMsg("\n");

  return HAL_OK;
}

static HAL_StatusTypeDef K9F1G08U0E_Format(NAND_HandleTypeDef *hnand,
  NandMetadata *nand_metadata)
{
  PrintMsg("Formatting NAND...\n");

  HAL_StatusTypeDef status = HAL_OK;
  if ((status = EraseUserBlocks(hnand, nand_metadata)) != HAL_OK)
  {
    return status;
  }
  PrintMsg("Make initial address translation table\n");
  memset(nand_metadata->att, 0, nand_metadata->att_size);
  if ((status = StoreNandMetadata(hnand, nand_metadata)) != HAL_OK)
  {
    return status;
  }

  PrintMsg("Formatting done.\n");

  return HAL_OK;
}

static HAL_StatusTypeDef StoreNandMetadata(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata)
{
  HAL_StatusTypeDef status = HAL_OK;
  if ((status = StoreAddrTranslationTable(hnand, nand_metadata)) != HAL_OK)
  {
    return status;
  }
  if ((status = StoreNandInnerConfiguration(hnand, nand_metadata)) != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef LoadNandMetadata(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata)
{
  // Get data in reverse order compare to StoreNandMetadata()
  HAL_StatusTypeDef status = HAL_OK;
  if ((status = LoadNandInnerConfiguration(hnand, nand_metadata)) != HAL_OK)
  {
    return status;
  }
  if ((status = LoadAddrTranslationTable(hnand, nand_metadata)) != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef StoreNandInnerConfiguration(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata)
{
  HAL_StatusTypeDef status = HAL_OK;
  NAND_AddressTypeDef nand_addr = {0};

  PrintMsg("Store inner configuration to NAND\n");

  for (nand_addr.Block = NAND_INNER_CONFIGURATION_BLOCK_INDEX;
       nand_addr.Block > nand_metadata->att_addr.Block; nand_addr.Block--)
  {
    if ((status = K9F1G08U0E_Erase_Block(hnand, &nand_addr)) != HAL_OK)
    {
      if (status == HAL_ERROR)
      {
        SetNandBlockInvalid(nand_metadata->ibt, nand_metadata->ibt_size,
          &nand_addr);
        // TODO: Mark block as invalid in NAND memory
      }
      continue;
    }

    if (SerializeNandInnerConfiguration(nand_metadata,
        g_nand_read_write_buffer, sizeof(g_nand_read_write_buffer)) != 0)
    {
      return HAL_ERROR;
    }
    if ((status = HAL_NAND_Write_Page_8b(hnand, &nand_addr,
        g_nand_read_write_buffer, 1)) != HAL_OK)
    {
      if (status == HAL_ERROR)
      {
        SetNandBlockInvalid(nand_metadata->ibt, nand_metadata->ibt_size,
          &nand_addr);
        // TODO: Copy invalid block pages to the correct block
      }
      continue;
    }

    break;
  }
  if (nand_addr.Block == nand_metadata->att_addr.Block)
  {
    PrintMsg("StoreNandInnerConfiguration() error: no valid block for inner configuration\n");

    return HAL_ERROR;
  }

  nand_metadata->inner_config_addr = nand_addr;

  return HAL_OK;
}

static HAL_StatusTypeDef LoadNandInnerConfiguration(
    NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata)
{
  HAL_StatusTypeDef status = HAL_OK;

  PrintMsg("Load inner configuration from NAND\n");

  // TODO: Handle the case, when a block with the index
  // NAND_INNER_CONFIGURATION_BLOCK_INDEX is invalid
  NAND_AddressTypeDef nand_addr = {0};
  nand_addr.Block = NAND_INNER_CONFIGURATION_BLOCK_INDEX;
  if ((status = HAL_NAND_Read_Page_8b(hnand, &nand_addr,
      g_nand_read_write_buffer, 1)) != HAL_OK)
  {
    return status;
  }

  if (DeserializeNandInnerConfiguration(g_nand_read_write_buffer,
    sizeof(g_nand_read_write_buffer), nand_metadata) != 0)
  {
    return HAL_ERROR;
  }

  nand_metadata->inner_config_addr = nand_addr;

  return HAL_OK;
}

static HAL_StatusTypeDef StoreAddrTranslationTable(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata)
{
  HAL_StatusTypeDef status = HAL_OK;
  NAND_AddressTypeDef nand_addr = {0};

  PrintMsg("Store address translation table to NAND\n");

  for (nand_addr.Block = nand_metadata->att_addr.Block;
       nand_addr.Block < nand_metadata->inner_config_addr.Block; nand_addr.Block++)
  {
    if ((status = K9F1G08U0E_Erase_Block(hnand, &nand_addr)) != HAL_OK)
    {
      if (status == HAL_ERROR)
      {
        SetNandBlockInvalid(nand_metadata->ibt, nand_metadata->ibt_size,
          &nand_addr);
        // TODO: Mark block as invalid in NAND memory
      }
      continue;
    }
    if ((status = HAL_NAND_Write_Page_8b(hnand, &nand_addr,
        (uint8_t*)nand_metadata->att, nand_metadata->att_size / NAND_PAGE_SIZE)) != HAL_OK)
    {
      if (status == HAL_ERROR)
      {
        SetNandBlockInvalid(nand_metadata->ibt, nand_metadata->ibt_size,
          &nand_addr);
        // TODO: Copy invalid block pages to the correct block
      }
      continue;
    }

    break;
  }
  if (nand_addr.Block == nand_metadata->inner_config_addr.Block)
  {
    PrintMsg("Error: no valid block for address translation table\n");

    return HAL_ERROR;
  }
  nand_metadata->att_addr = nand_addr;

  return HAL_OK;
}

static HAL_StatusTypeDef LoadAddrTranslationTable(
    NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata)
{
  HAL_StatusTypeDef status = HAL_OK;

  PrintMsg("Load address translation table from NAND\n");

  if ((status = HAL_NAND_Read_Page_8b(hnand, &nand_metadata->att_addr,
      (uint8_t*)nand_metadata->att, nand_metadata->att_size / NAND_PAGE_SIZE)) != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef EraseUserBlocks(
  NAND_HandleTypeDef *hnand, NandMetadata *nand_metadata)
{
  HAL_StatusTypeDef status = HAL_OK;
  NAND_AddressTypeDef nand_addr = {0};

  PrintMsg("Erase user blocks\n");
  int free_page_addr_set = 0;
  for (nand_addr.Block = 0; nand_addr.Block < NAND_USED_BLOCK_NUMBER;
    nand_addr.Block++)
  {
    if ((status = K9F1G08U0E_Erase_Block(hnand, &nand_addr)) != HAL_OK)
    {
      if (status == HAL_ERROR)
      {
        SetNandBlockInvalid(nand_metadata->ibt, nand_metadata->ibt_size,
          &nand_addr);
      }
      continue;
    }

    if (!free_page_addr_set)
    {
      nand_metadata->free_page_addr = nand_addr;
      free_page_addr_set = 1;
    }
  }

  if (!free_page_addr_set)
  {
    PrintMsg("EraseUserBlocks() error: no valid user blocks\n");

    return HAL_ERROR;
  }

  return HAL_OK;
}


static HAL_StatusTypeDef K9F1G08U0E_Erase_Block(
  NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *nand_addr)
{
  PrintMsg("Erasing block... ");
  PrintNandAddr(nand_addr);

  HAL_StatusTypeDef status = HAL_NAND_Erase_Block(hnand, nand_addr);
  if (status != HAL_OK)
  {
    sprintf(g_uart_msg, " Error. HAL status: %d\n", status);
    PrintMsg(g_uart_msg);

    return status;
  }

  /* Get tick */
  uint32_t tickstart = HAL_GetTick();

  /* Read status until NAND is ready */
  uint32_t nand_status = NAND_READY;
  while (!((nand_status = HAL_NAND_Read_Status(hnand)) & NAND_READY))
  {
    if ((HAL_GetTick() - tickstart) > NAND_WRITE_TIMEOUT)
    {
      PrintMsg("Erase Timeout\n");
      return HAL_TIMEOUT;
    }
  }

  if (nand_status & NAND_ERROR)
  {
    PrintMsg(" Error\n");
    return HAL_ERROR;
  }

  sprintf(g_uart_msg, " Ok. time: %ld us\n", HAL_GetTick() - tickstart);
  PrintMsg(g_uart_msg);

  return HAL_OK;
}

static int SerializeNandInnerConfiguration(
    const NandMetadata *nand_metadata, uint8_t *buf, size_t buf_size)
{
  if ((2 * sizeof(NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE) + nand_metadata->ibt_size) > buf_size)
  {
    PrintMsg("SerializeNandInnerConfiguration() error: buf is too small\n");

    return 1;
  }

  memset(buf, 0, sizeof(buf_size));
  uint8_t *p_in_buf = buf;
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE att_entry;
  // Serialize user first free page address
  GetNandAddrAddrTranslationTableEntry(&nand_metadata->free_page_addr, &att_entry);
  memcpy(p_in_buf, &att_entry, sizeof(att_entry));
  p_in_buf += sizeof(att_entry);
  // Serialize address translation table address
  GetNandAddrAddrTranslationTableEntry(&nand_metadata->att_addr, &att_entry);
  memcpy(p_in_buf, &att_entry, sizeof(att_entry));
  p_in_buf += sizeof(att_entry);
  // Serialize invalid block table
  memcpy(p_in_buf, nand_metadata->ibt, nand_metadata->ibt_size);
  p_in_buf += nand_metadata->ibt_size;

  return 0;
}

static int DeserializeNandInnerConfiguration(
    const uint8_t *buf, size_t buf_size, NandMetadata *nand_metadata)
{
  if ((2 * sizeof(NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE) + nand_metadata->ibt_size) > buf_size)
  {
    PrintMsg("DeserializeNandInnerConfiguration() error: buf is too small\n");

    return 1;
  }

  const uint8_t *p_in_buf = buf;
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE att_entry;

  // Deserialize user first free page address
  memcpy(&att_entry, p_in_buf, sizeof(att_entry));
  p_in_buf += sizeof(att_entry);
  GetAddrTranslationTableEntryNandAddr(att_entry, &nand_metadata->free_page_addr);
  // Deserialize address translation table address
  memcpy(&att_entry, p_in_buf, sizeof(att_entry));
  p_in_buf += sizeof(att_entry);
  GetAddrTranslationTableEntryNandAddr(att_entry, &nand_metadata->att_addr);
  // Deserialize invalid block table
  memcpy(nand_metadata->ibt, p_in_buf, nand_metadata->ibt_size);
  p_in_buf += nand_metadata->ibt_size;

  return 0;
}

// Invalid block table

static int IsNandBlockInvalid(const uint8_t *ibt, size_t ibt_size,
  const NAND_AddressTypeDef *nand_addr)
{
  if ((nand_addr->Block / BITS_IN_BYTE) >= ibt_size)
  {
    PrintMsg("IsNandBlockInvalid() error: block number is out of range\n");
    return 0;
  }

  return (int)((ibt[nand_addr->Block / BITS_IN_BYTE]
      >> (nand_addr->Block % BITS_IN_BYTE)) & 0x01);
}

static void SetNandBlockInvalid(uint8_t *ibt, size_t ibt_size,
  const NAND_AddressTypeDef *nand_addr)
{
  if ((nand_addr->Block / BITS_IN_BYTE) >= ibt_size)
  {
    PrintMsg("SetNandBlockInvalid() error: block number is out of range\n");
    return;
  }

  sprintf(g_uart_msg, "Mark NAND block as invalid: %d\n", nand_addr->Block);
  PrintMsg(g_uart_msg);

  ibt[nand_addr->Block / BITS_IN_BYTE] =
    (uint8_t)(1 << (nand_addr->Block % BITS_IN_BYTE));
}

// Address translation table

static int SetAddrTranslationTableNandAddr(
  NandMetadata *nand_metadata,
  uint32_t lba, const NAND_AddressTypeDef *nand_addr)
{
  // assert
  if (lba >= NAND_USED_PAGE_NUMBER)
  {
    sprintf(g_uart_msg, "SetAddrTranlationTableNandAddr() error: "
      "lba is out of range. lba: %ld, max_lba: %d\n",
      lba, NAND_USED_PAGE_NUMBER - 1);
    PrintMsg(g_uart_msg);

    return 1;
  }

  nand_metadata->att[lba] =
    NandAddrToAddrTranslationTableEntry(nand_addr);

  // Log
  // sprintf(g_uart_msg, "SetAddrTranlationTableNandAddr(): lba: %ld, entry: 0x%04X\n",
  //   lba, nand_metadata->att[lba]);
  sprintf(g_uart_msg, "SetAddrTranlationTableNandAddr(): lba: %ld, ", lba);
  PrintMsg(g_uart_msg);
  PrintNandAddr(nand_addr);
  sprintf(g_uart_msg, ", entry: 0x%04X\n", nand_metadata->att[lba]);
  PrintMsg(g_uart_msg);

  return 0;
}

static int GetAddrTranslationTableNandAddr(
  const NandMetadata *nand_metadata,
  uint32_t lba, NAND_AddressTypeDef* nand_addr)
{
  // assert
  if (lba >= NAND_USED_PAGE_NUMBER)
  {
    sprintf(g_uart_msg, "GetAddrTranlationTableNandAddr() error: "
      "lba is out of range. lba: %ld, max_lba: %d\n",
      lba, NAND_USED_PAGE_NUMBER - 1);
    PrintMsg(g_uart_msg);

    return 1;
  }

  // if (entry == 0)
  // {
  //   LbaBlockAddrToNandAddr(blk_addr, hnand, nand_addr);
  // }
  // else
  // {
    GetAddrTranslationTableEntryNandAddr(nand_metadata->att[lba], nand_addr);
  // }

  // sprintf(g_uart_msg, "GetAddrTranlationTableNandAddr(): lba: %ld, entry: 0x%04X, ",
  //   lba, nand_metadata->att[lba]);
  // PrintMsg(g_uart_msg);
  // PrintNandAddr(nand_addr);
  // PrintMsg("\n");

  return 0;
}

static void GetAddrTranslationTableEntryNandAddr(
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE att_entry,
  NAND_AddressTypeDef* nand_addr)
{
  nand_addr->Plane = 0;
  nand_addr->Block = ((att_entry & NAND_ADDR_TRANSLATION_TABLE_ENTRY_BLOCK_BITS_MASK)
      >> NAND_ADDR_TRANSLATION_TABLE_ENTRY_PAGE_BITS_COUNT);
  nand_addr->Page = (att_entry & NAND_ADDR_TRANSLATION_TABLE_ENTRY_PAGE_BITS_MASK);
}

static NAND_AddressTypeDef AddrTranslationTableEntryToNandAddr(
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE att_entry)
{
  NAND_AddressTypeDef nand_addr;
  GetAddrTranslationTableEntryNandAddr(att_entry, &nand_addr);

  return nand_addr;
}

static void GetNandAddrAddrTranslationTableEntry(
  const NAND_AddressTypeDef* nand_addr,
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE* att_entry)
{
  *att_entry = ((nand_addr->Block << NAND_ADDR_TRANSLATION_TABLE_ENTRY_PAGE_BITS_COUNT)
      | nand_addr->Page);
}

static NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE NandAddrToAddrTranslationTableEntry(
    const NAND_AddressTypeDef *nand_addr)
{
  NAND_ADDR_TRANSLATION_TABLE_ENTRY_TYPE att_entry;
  GetNandAddrAddrTranslationTableEntry(nand_addr, &att_entry);

  return att_entry;
}

// LBA to NAND address conversion

static void LbaToNandAddr(NAND_HandleTypeDef *hnand,
  uint32_t lba, NAND_AddressTypeDef *nand_addr)
{
  uint32_t pagesInPlane = hnand->Config.PlaneSize * hnand->Config.BlockSize;
  nand_addr->Plane = lba / pagesInPlane;
  lba %= pagesInPlane;
  uint32_t pagesInBlock = hnand->Config.BlockSize;
  nand_addr->Block = lba / pagesInBlock;
  nand_addr->Page = lba % pagesInBlock;
}

// Log

static void PrintMsg(const char* msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
}

// static void PrintBlkAddr(uint32_t blk_addr)
// {
//   sprintf(g_uart_msg, "blk_addr: %ld\n", blk_addr);
//   HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
// }

static void PrintNandAddr(const NAND_AddressTypeDef *nand_addr)
{
  sprintf(g_uart_msg, "NAND addr. plane: %d, block: %d, page=%d",
    nand_addr->Plane, nand_addr->Block, nand_addr->Page);
  PrintMsg(g_uart_msg);
}

// static void PrintNandOpStatus(NAND_AddressTypeDef *nand_addr)
// {
//   sprintf(g_uart_msg, "Nand Addr. plane: %d, block: %d, page=%d\n",
//           nand_addr->Plane, nand_addr->Block, nand_addr->Page);
//   HAL_UART_Transmit(&huart1, (uint8_t *)g_uart_msg, strlen(g_uart_msg), 1000);
// }

static void PrintNandBuffer(const uint8_t *buf)
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

static void PrintNandBufferSpareArea(const uint8_t *buf)
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

static void PrintBuffer(const uint8_t *buf, size_t buf_size)
{
  for (size_t i = 0; i < buf_size; i++)
  {
    if (i % 16 == 0)
      PrintMsg("\n");
    else if (i % 8 == 0)
      PrintMsg(" ");

    sprintf(g_uart_msg, "%02X ", buf[i]);
    PrintMsg(g_uart_msg);
  }
}


// static HAL_StatusTypeDef K9F1G08U0E_My_HAL_NAND_Read_Page_8b(
//   NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumPageToRead)
// {
//   uint32_t index;
//   uint32_t tickstart;
//   uint32_t deviceAddress, numPagesRead = 0U, nandAddress, nbpages = NumPageToRead;
//   uint8_t *buff = pBuffer;

//   /* Check the NAND controller state */
//   if (hnand->State == HAL_NAND_STATE_BUSY)
//   {
//     return HAL_BUSY;
//   }
//   else if (hnand->State == HAL_NAND_STATE_READY)
//   {
//     /* Process Locked */
//     __HAL_LOCK(hnand);

//     /* Update the NAND controller state */
//     hnand->State = HAL_NAND_STATE_BUSY;

//     /* Identify the device address */
//     if (hnand->Init.NandBank == FSMC_NAND_BANK2)
//     {
//       deviceAddress = NAND_DEVICE1;
//     }
//     else
//     {
//       deviceAddress = NAND_DEVICE2;
//     }

//     /* NAND raw address calculation */
//     nandAddress = ARRAY_ADDRESS(pAddress, hnand);

//     /* Page(s) read loop */
//     while ((nbpages != 0U) && (nandAddress < ((hnand->Config.BlockSize) * (hnand->Config.BlockNbr))))
//     {
//       /* Send read page command sequence */
//       *(__IO uint8_t *)((uint32_t)(deviceAddress | CMD_AREA)) = NAND_CMD_AREA_A;
//       __DSB();

//       *(__IO uint8_t *)((uint32_t)(deviceAddress | ADDR_AREA)) = 0x00U;
//       __DSB();
//       *(__IO uint8_t *)((uint32_t)(deviceAddress | ADDR_AREA)) = 0x00U;
//       __DSB();
//       *(__IO uint8_t *)((uint32_t)(deviceAddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandAddress);
//       __DSB();
//       *(__IO uint8_t *)((uint32_t)(deviceAddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandAddress);
//       __DSB();

//       *(__IO uint8_t *)((uint32_t)(deviceAddress | ATTRIBUTE_AREA | CMD_AREA)) = NAND_CMD_AREA_TRUE1;
//       __DSB();

//       if (hnand->Config.ExtraCommandEnable == ENABLE)
//       {
//         /* Get tick */
//         tickstart = HAL_GetTick();

//         /* Read status until NAND is ready */
//         while (HAL_NAND_Read_Status(hnand) != NAND_READY)
//         {
//           if ((HAL_GetTick() - tickstart) > NAND_WRITE_TIMEOUT)
//           {
//             /* Update the NAND controller state */
//             hnand->State = HAL_NAND_STATE_ERROR;

//             /* Process unlocked */
//             __HAL_UNLOCK(hnand);

//             return HAL_TIMEOUT;
//           }
//         }

//         /* Go back to read mode */
//         *(__IO uint8_t *)((uint32_t)(deviceAddress | CMD_AREA)) = ((uint8_t)0x00U);
//         __DSB();
//       }

//       /* Get Data into Buffer */
//       for (index = 0U; index < hnand->Config.PageSize; index++)
//       {
//         *buff = *(uint8_t *)deviceAddress;
//         buff++;
//       }

//       /* Increment read pages number */
//       numPagesRead++;

//       /* Decrement pages to read */
//       nbpages--;

//       /* Increment the NAND address */
//       nandAddress = (uint32_t)(nandAddress + 1U);
//     }

//     /* Update the NAND controller state */
//     hnand->State = HAL_NAND_STATE_READY;

//     /* Process unlocked */
//     __HAL_UNLOCK(hnand);
//   }
//   else
//   {
//     return HAL_ERROR;
//   }

//   return HAL_OK;
// }

// ///////////

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

// //////////////

// static HAL_StatusTypeDef K9F1G08U0E_Erase_Block(uint32_t nand_blk_addr)
// {
//   HAL_StatusTypeDef nand_op_state = HAL_OK;

//   // PrintMsg("Erase block. ");
//   // PrintBlkAddr(blk_addr);

//   uint32_t nand_blk_addr = blk_addr / LOGICAL_PAGES_IN_NAND_PAGE;
//   // uint32_t logical_blk_in_nand_blk = blk_addr % LOGICAL_PAGES_IN_NAND_PAGE;
//   // sprintf(g_uart_msg, "nand_blk_addr: %ld, logical_blk_in_nand_blk: %ld\n",
//   //         nand_blk_addr, logical_blk_in_nand_blk);
//   // PrintMsg(g_uart_msg);

//   NAND_AddressTypeDef nand_addr;
//   // GetConvertTableNandAddr(nand_blk_addr, &hnand1, &nand_addr);
//   LbaBlockAddrToNandAddr(nand_blk_addr, &hnand1, &nand_addr);

//   nand_op_state = HAL_NAND_Erase_Block(&hnand1, &nand_addr);
//   // sprintf(g_uart_msg, "Erase status: %d\n", nand_op_state);
//   // PrintMsg(g_uart_msg);
//   if (nand_op_state != HAL_OK)
//   {
//     return (USBD_FAIL);
//   }

//   uint32_t tickstart;
//   /* Get tick */
//   tickstart = HAL_GetTick();

//   /* Read status until NAND is ready */
//   uint32_t status;
//   while ((status = HAL_NAND_Read_Status(&hnand1)) != NAND_READY)
//   {
//     if ((HAL_GetTick() - tickstart) > NAND_WRITE_TIMEOUT)
//     {
//       // PrintMsg("Erase Timeout\n");
//       return (USBD_FAIL);
//     }
//   }
//   // sprintf(g_uart_msg, "Erase Ok. time: %ld, status: 0x%lX\n", HAL_GetTick() - tickstart, status);
//   // PrintMsg(g_uart_msg);

//   return (USBD_OK);
// }

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
