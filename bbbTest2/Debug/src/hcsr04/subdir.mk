################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/hcsr04/hcsr04.cpp 

OBJS += \
./src/hcsr04/hcsr04.o 

CPP_DEPS += \
./src/hcsr04/hcsr04.d 


# Each subdirectory must supply rules for building sources it contributes
src/hcsr04/%.o: ../src/hcsr04/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++-4.7 -I/usr/arm-linux-gnueabihf/include/c++/4.7.3 -I/usr/arm-linux-gnueabihf/include/pruss -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


