################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/RadioSystem/Telosb/TelosbRadioSystem.cpp 

C_SRCS += \
../src/RadioSystem/Telosb/serialsource.c 

OBJS += \
./src/RadioSystem/Telosb/TelosbRadioSystem.o \
./src/RadioSystem/Telosb/serialsource.o 

C_DEPS += \
./src/RadioSystem/Telosb/serialsource.d 

CPP_DEPS += \
./src/RadioSystem/Telosb/TelosbRadioSystem.d 


# Each subdirectory must supply rules for building sources it contributes
src/RadioSystem/Telosb/%.o: ../src/RadioSystem/Telosb/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -Iusr/local/include -I/usr/local/include/opencv -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/src" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/src/ASN.1" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/src/MultimediaSystem/includes" -I/usr/include -O0 -g3 -Wall -c -fmessage-length=0 -g2 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/RadioSystem/Telosb/%.o: ../src/RadioSystem/Telosb/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/greeneyes/OpenCV-Install/2.4.4/include -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/src/ASN.1" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/src" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/src/MultimediaSystem/includes" -I/usr/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


