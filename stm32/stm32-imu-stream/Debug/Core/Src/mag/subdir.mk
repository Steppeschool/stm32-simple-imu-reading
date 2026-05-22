################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/mag/hmc5883l_drv.c \
../Core/Src/mag/lis3mdl_drv.c \
../Core/Src/mag/mmc5983ma_drv.c \
../Core/Src/mag/qmc5883l_drv.c 

OBJS += \
./Core/Src/mag/hmc5883l_drv.o \
./Core/Src/mag/lis3mdl_drv.o \
./Core/Src/mag/mmc5983ma_drv.o \
./Core/Src/mag/qmc5883l_drv.o 

C_DEPS += \
./Core/Src/mag/hmc5883l_drv.d \
./Core/Src/mag/lis3mdl_drv.d \
./Core/Src/mag/mmc5983ma_drv.d \
./Core/Src/mag/qmc5883l_drv.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/mag/%.o Core/Src/mag/%.su Core/Src/mag/%.cyclo: ../Core/Src/mag/%.c Core/Src/mag/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -DARM_MATH_CM4 -c -I../Core/Inc -I../Core/Inc/imu -I../Core/Inc/mag -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-mag

clean-Core-2f-Src-2f-mag:
	-$(RM) ./Core/Src/mag/hmc5883l_drv.cyclo ./Core/Src/mag/hmc5883l_drv.d ./Core/Src/mag/hmc5883l_drv.o ./Core/Src/mag/hmc5883l_drv.su ./Core/Src/mag/lis3mdl_drv.cyclo ./Core/Src/mag/lis3mdl_drv.d ./Core/Src/mag/lis3mdl_drv.o ./Core/Src/mag/lis3mdl_drv.su ./Core/Src/mag/mmc5983ma_drv.cyclo ./Core/Src/mag/mmc5983ma_drv.d ./Core/Src/mag/mmc5983ma_drv.o ./Core/Src/mag/mmc5983ma_drv.su ./Core/Src/mag/qmc5883l_drv.cyclo ./Core/Src/mag/qmc5883l_drv.d ./Core/Src/mag/qmc5883l_drv.o ./Core/Src/mag/qmc5883l_drv.su

.PHONY: clean-Core-2f-Src-2f-mag

