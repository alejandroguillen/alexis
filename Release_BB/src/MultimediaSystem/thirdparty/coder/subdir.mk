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
	arm-linux-gnueabihf-g++ -I/home/alexis/BeagleBone/gsl-arm -I/home/alexis/BeagleBone/boost-arm -I/home/alexis/BeagleBone/gsl-arm/include -I/home/alexis/BeagleBone/lpsolve-arm -I/home/alexis/BeagleBone/opencv-arm/include -I/home/alexis/BeagleBone/opencv-arm -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/ASN.1" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/MultimediaSystem/includes" -O0 -g3 -Wall -c -fmessage-length=0 -g2 -mfpu=neon -mfloat-abi=hard -flax-vector-conversions -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


