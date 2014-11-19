################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/mpu6050/I2Cdev.cpp \
../src/mpu6050/MPU6050.cpp 

OBJS += \
./src/mpu6050/I2Cdev.o \
./src/mpu6050/MPU6050.o 

CPP_DEPS += \
./src/mpu6050/I2Cdev.d \
./src/mpu6050/MPU6050.d 


# Each subdirectory must supply rules for building sources it contributes
src/mpu6050/%.o: ../src/mpu6050/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++-4.7 -I/usr/arm-linux-gnueabihf/include/c++/4.7.3 -I/usr/arm-linux-gnueabihf/include/pruss -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


