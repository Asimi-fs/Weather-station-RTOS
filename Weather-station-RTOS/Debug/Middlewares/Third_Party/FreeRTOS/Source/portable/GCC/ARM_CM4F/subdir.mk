################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/BME680" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/Config" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/e-Paper" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/Examples" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/Fonts" -I"C:/Users/Asimi/STM32CubeIDE/workspace_1.5.1/Weather-station-RTOS/Core/E-paper/GUI" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

