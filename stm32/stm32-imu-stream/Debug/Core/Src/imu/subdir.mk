################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/imu/bmi270_config.c \
../Core/Src/imu/bmi270_drv.c \
../Core/Src/imu/icm20948_drv.c \
../Core/Src/imu/icm42688_drv.c \
../Core/Src/imu/ism330dhcx_drv.c \
../Core/Src/imu/lsm6dso_drv.c \
../Core/Src/imu/mpu6050_drv.c 

OBJS += \
./Core/Src/imu/bmi270_config.o \
./Core/Src/imu/bmi270_drv.o \
./Core/Src/imu/icm20948_drv.o \
./Core/Src/imu/icm42688_drv.o \
./Core/Src/imu/ism330dhcx_drv.o \
./Core/Src/imu/lsm6dso_drv.o \
./Core/Src/imu/mpu6050_drv.o 

C_DEPS += \
./Core/Src/imu/bmi270_config.d \
./Core/Src/imu/bmi270_drv.d \
./Core/Src/imu/icm20948_drv.d \
./Core/Src/imu/icm42688_drv.d \
./Core/Src/imu/ism330dhcx_drv.d \
./Core/Src/imu/lsm6dso_drv.d \
./Core/Src/imu/mpu6050_drv.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/imu/%.o Core/Src/imu/%.su Core/Src/imu/%.cyclo: ../Core/Src/imu/%.c Core/Src/imu/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -DARM_MATH_CM4 -c -I../Core/Inc -I../Core/Inc/imu -I../Core/Inc/mag -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-imu

clean-Core-2f-Src-2f-imu:
	-$(RM) ./Core/Src/imu/bmi270_config.cyclo ./Core/Src/imu/bmi270_config.d ./Core/Src/imu/bmi270_config.o ./Core/Src/imu/bmi270_config.su ./Core/Src/imu/bmi270_drv.cyclo ./Core/Src/imu/bmi270_drv.d ./Core/Src/imu/bmi270_drv.o ./Core/Src/imu/bmi270_drv.su ./Core/Src/imu/icm20948_drv.cyclo ./Core/Src/imu/icm20948_drv.d ./Core/Src/imu/icm20948_drv.o ./Core/Src/imu/icm20948_drv.su ./Core/Src/imu/icm42688_drv.cyclo ./Core/Src/imu/icm42688_drv.d ./Core/Src/imu/icm42688_drv.o ./Core/Src/imu/icm42688_drv.su ./Core/Src/imu/ism330dhcx_drv.cyclo ./Core/Src/imu/ism330dhcx_drv.d ./Core/Src/imu/ism330dhcx_drv.o ./Core/Src/imu/ism330dhcx_drv.su ./Core/Src/imu/lsm6dso_drv.cyclo ./Core/Src/imu/lsm6dso_drv.d ./Core/Src/imu/lsm6dso_drv.o ./Core/Src/imu/lsm6dso_drv.su ./Core/Src/imu/mpu6050_drv.cyclo ./Core/Src/imu/mpu6050_drv.d ./Core/Src/imu/mpu6050_drv.o ./Core/Src/imu/mpu6050_drv.su

.PHONY: clean-Core-2f-Src-2f-imu

