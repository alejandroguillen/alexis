################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Tasks/Tasks.cpp 

OBJS += \
./src/Tasks/Tasks.o 

CPP_DEPS += \
./src/Tasks/Tasks.d 


# Each subdirectory must supply rules for building sources it contributes
src/Tasks/%.o: ../src/Tasks/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


