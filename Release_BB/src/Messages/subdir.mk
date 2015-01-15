################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Messages/ACKsliceMsg.cpp \
../src/Messages/CoopInfoMsg.cpp \
../src/Messages/DataATCMsg.cpp \
../src/Messages/DataCTAMsg.cpp \
../src/Messages/Header.cpp \
../src/Messages/StartATCMsg.cpp \
../src/Messages/StartCTAMsg.cpp \
../src/Messages/StartDATCMsg.cpp \
../src/Messages/StopMsg.cpp 

OBJS += \
./src/Messages/ACKsliceMsg.o \
./src/Messages/CoopInfoMsg.o \
./src/Messages/DataATCMsg.o \
./src/Messages/DataCTAMsg.o \
./src/Messages/Header.o \
./src/Messages/StartATCMsg.o \
./src/Messages/StartCTAMsg.o \
./src/Messages/StartDATCMsg.o \
./src/Messages/StopMsg.o 

CPP_DEPS += \
./src/Messages/ACKsliceMsg.d \
./src/Messages/CoopInfoMsg.d \
./src/Messages/DataATCMsg.d \
./src/Messages/DataCTAMsg.d \
./src/Messages/Header.d \
./src/Messages/StartATCMsg.d \
./src/Messages/StartCTAMsg.d \
./src/Messages/StartDATCMsg.d \
./src/Messages/StopMsg.d 


# Each subdirectory must supply rules for building sources it contributes
src/Messages/%.o: ../src/Messages/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I/home/alexis/BeagleBone/gsl-arm -I/home/alexis/BeagleBone/boost-arm -I/home/alexis/BeagleBone/gsl-arm/include -I/home/alexis/BeagleBone/lpsolve-arm -I/home/alexis/BeagleBone/opencv-arm/include -I/home/alexis/BeagleBone/opencv-arm -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/ASN.1" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/MultimediaSystem/includes" -O0 -g3 -Wall -c -fmessage-length=0 -g2 -mfpu=neon -mfloat-abi=hard -flax-vector-conversions -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


