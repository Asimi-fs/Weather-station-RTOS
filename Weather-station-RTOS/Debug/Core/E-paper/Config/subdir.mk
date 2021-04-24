################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/E-paper/Config/DEV_Config.c 

OBJS += \
./Core/E-paper/Config/DEV_Config.o 

C_DEPS += \
./Core/E-paper/Config/DEV_Config.d 


# Each subdirectory must supply rules for building sources it contributes
Core/E-paper/Config/DEV_Config.o: ../Core/E-paper/Config/DEV_Config.c Core/E-paper/Config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/BME680" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/Config" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/e-Paper" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/Examples" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/Fonts" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/GUI" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/E-paper/Config/DEV_Config.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

