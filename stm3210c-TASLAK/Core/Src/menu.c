/**
  ******************************************************************************
  * @file    USB_Host/MSC_Standalone/Src/menu.c 
  * @author  MCD Application Team
  * @brief   This file implements Menu Functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------ */
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"
#include "stm32f1xx_hal.h"
#include "lcd.h"
#include "lcd_log.h"
#include "lcd_log_conf.h"
/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
#define IMAGE_BUFFER_SIZE    512
/**
* @}
*/


/** @defgroup USBH_USR_Private_Macros
* @{
*/
/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
extern ApplicationTypeDef Appli_state;
KeyCountTypeDef Key = IDLE;
/**
* @}
*/


/** @defgroup USBH_USR_Private_Variables
* @{
*/
uint8_t filenameString[15] = { 0 };

FATFS fatfs;
FIL file;
uint8_t Image_Buf[IMAGE_BUFFER_SIZE];
uint8_t line_idx = 0;
/* Private function prototypes ----------------------------------------------- */
void MSC_MenuProcess(void);
void Key_Press(void);
static uint8_t Explore_Disk(char *path, uint8_t recu_level);
/* Private functions --------------------------------------------------------- */
/**
  * @brief  Manages MSC Menu Process.
  * @param  None
  * @retval None
  */
void MSC_MenuProcess(void)
{
    /* Display disk content */
    if (Appli_state == APPLICATION_READY)
    {
    	Key_Press();
    }
}

void Key_Press(void)
{
		switch(Key)
		{
		 case IDLE:
			 if(BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
			 {
				 Key = WRITE_FILE;
				 if(UsbTest_Write()) LCD_UsrLog("New Text file created.\n");
				 else LCD_UsrLog("Text file couldn't created.\n");
			     BSP_LED_On(LED2);
			     HAL_Delay(1000);
			     BSP_LED_Off(LED2);
				 BSP_LCD_ClearStringLine(17);
			 }
			    BSP_LCD_DisplayStringAtLine(17,
			                              (uint8_t *) "Press Key to write a New Text File...");
			    BSP_LCD_SetTextColor(LCD_COLOR_RED);
			 break;
		 case WRITE_FILE:
			 if(BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
			 {
				 Key = DISK_CONTENT;
				 Explore_Disk("0:/", 1);
				 BSP_LCD_ClearStringLine(17);
			 }
			 				BSP_LCD_DisplayStringAtLine(17,
			 										  (uint8_t *) "Press Key to Explore Disk Content...");
			 				BSP_LCD_SetTextColor(LCD_COLOR_RED);
			 break;
		 case DISK_CONTENT:
			 if(BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
			 {
				 Key = IDLE;
				 if(UsbTest_Read() == FALSE) LCD_UsrLog("Text file couldn't read.\n");
				 BSP_LCD_ClearStringLine(17);
			 }
			     BSP_LCD_DisplayStringAtLine(17,
			                              (uint8_t *) "Press Key to read last Text File...");
			     BSP_LCD_SetTextColor(LCD_COLOR_RED);
			 break;
		 default:
			 break;
		}
}
/* USER CODE END 4 */

/**
* @brief  Explore_Disk
*         Displays disk content
* @param  path: pointer to root path
* @retval None
*/
static uint8_t Explore_Disk(char *path, uint8_t recu_level)
{

  FRESULT res;
  FILINFO fno;
  DIR dir;
  char *fn;
  char tmp[14];

  res = f_opendir(&dir, path);
  if (res == FR_OK)
  {
    while (Appli_state == APPLICATION_READY)
    {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }
      if (fno.fname[0] == '.')
      {
        continue;
      }

      fn = fno.fname;
      strcpy(tmp, fn);

      line_idx++;
      if (line_idx > 9)
      {
        line_idx = 0;
      }

      if (recu_level == 1)
      {
        LCD_UsrLog("   |__");
      }
      else if (recu_level == 2)
      {
        LCD_UsrLog("   |   |__");
      }
      if ((fno.fattrib & AM_MASK) == AM_DIR)
      {
        strcat(tmp, "\n");
        LCD_UsrLog((void *)tmp);
      }
      else
      {
        strcat(tmp, "\n");
        LCD_UsrLog((void *)tmp);
      }

      if (((fno.fattrib & AM_MASK) == AM_DIR) && (recu_level == 1))
      {
        Explore_Disk(fn, 2);
      }
    }
  }
  return res;
}

/**
* @brief  Show_Image
*         Displays BMP image
* @param  None
* @retval None
*/



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
