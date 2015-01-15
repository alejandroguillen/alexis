################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/RadioSystem/WiFi/ALWiFiRadioSystem.cpp \
../src/RadioSystem/WiFi/Connection.cpp \
../src/RadioSystem/WiFi/ConnectionManager.cpp \
../src/RadioSystem/WiFi/WiFiRadioSystem.cpp 

OBJS += \
./src/RadioSystem/WiFi/ALWiFiRadioSystem.o \
./src/RadioSystem/WiFi/Connection.o \
./src/RadioSystem/WiFi/ConnectionManager.o \
./src/RadioSystem/WiFi/WiFiRadioSystem.o 

CPP_DEPS += \
./src/RadioSystem/WiFi/ALWiFiRadioSystem.d \
./src/RadioSystem/WiFi/Connection.d \
./src/RadioSystem/WiFi/ConnectionManager.d \
./src/RadioSystem/WiFi/WiFiRadioSystem.d 


# Each subdirectory must supply rules for building sources it contributes
src/RadioSystem/WiFi/%.o: ../src/RadioSystem/WiFi/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -Iusr/local/include -I/usr/local/include/opencv -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/ASN.1" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/MultimediaSystem/includes" -I/usr/include -O0 -g3 -Wall -c -fmessage-length=0 -g2 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


