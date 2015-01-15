################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/MultimediaSystem/thirdparty/coder/ac_extended.cpp 

OBJS += \
./src/MultimediaSystem/thirdparty/coder/ac_extended.o 

CPP_DEPS += \
./src/MultimediaSystem/thirdparty/coder/ac_extended.d 


# Each subdirectory must supply rules for building sources it contributes
src/MultimediaSystem/thirdparty/coder/%.o: ../src/MultimediaSystem/thirdparty/coder/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


