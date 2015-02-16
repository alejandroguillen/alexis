################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/NodeManager/NodeManager.cpp \
../src/NodeManager/SIMULATIONManager.cpp 

OBJS += \
./src/NodeManager/NodeManager.o \
./src/NodeManager/SIMULATIONManager.o 

CPP_DEPS += \
./src/NodeManager/NodeManager.d \
./src/NodeManager/SIMULATIONManager.d 


# Each subdirectory must supply rules for building sources it contributes
src/NodeManager/%.o: ../src/NodeManager/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


