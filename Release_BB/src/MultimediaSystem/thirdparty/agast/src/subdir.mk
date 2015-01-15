################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../src/MultimediaSystem/thirdparty/agast/src/AstDetector.cc \
../src/MultimediaSystem/thirdparty/agast/src/agast5_8.cc \
../src/MultimediaSystem/thirdparty/agast/src/agast5_8_nms.cc \
../src/MultimediaSystem/thirdparty/agast/src/agast7_12d.cc \
../src/MultimediaSystem/thirdparty/agast/src/agast7_12d_nms.cc \
../src/MultimediaSystem/thirdparty/agast/src/agast7_12s.cc \
../src/MultimediaSystem/thirdparty/agast/src/agast7_12s_nms.cc \
../src/MultimediaSystem/thirdparty/agast/src/nonMaximumSuppression.cc \
../src/MultimediaSystem/thirdparty/agast/src/oast9_16.cc \
../src/MultimediaSystem/thirdparty/agast/src/oast9_16_nms.cc 

OBJS += \
./src/MultimediaSystem/thirdparty/agast/src/AstDetector.o \
./src/MultimediaSystem/thirdparty/agast/src/agast5_8.o \
./src/MultimediaSystem/thirdparty/agast/src/agast5_8_nms.o \
./src/MultimediaSystem/thirdparty/agast/src/agast7_12d.o \
./src/MultimediaSystem/thirdparty/agast/src/agast7_12d_nms.o \
./src/MultimediaSystem/thirdparty/agast/src/agast7_12s.o \
./src/MultimediaSystem/thirdparty/agast/src/agast7_12s_nms.o \
./src/MultimediaSystem/thirdparty/agast/src/nonMaximumSuppression.o \
./src/MultimediaSystem/thirdparty/agast/src/oast9_16.o \
./src/MultimediaSystem/thirdparty/agast/src/oast9_16_nms.o 

CC_DEPS += \
./src/MultimediaSystem/thirdparty/agast/src/AstDetector.d \
./src/MultimediaSystem/thirdparty/agast/src/agast5_8.d \
./src/MultimediaSystem/thirdparty/agast/src/agast5_8_nms.d \
./src/MultimediaSystem/thirdparty/agast/src/agast7_12d.d \
./src/MultimediaSystem/thirdparty/agast/src/agast7_12d_nms.d \
./src/MultimediaSystem/thirdparty/agast/src/agast7_12s.d \
./src/MultimediaSystem/thirdparty/agast/src/agast7_12s_nms.d \
./src/MultimediaSystem/thirdparty/agast/src/nonMaximumSuppression.d \
./src/MultimediaSystem/thirdparty/agast/src/oast9_16.d \
./src/MultimediaSystem/thirdparty/agast/src/oast9_16_nms.d 


# Each subdirectory must supply rules for building sources it contributes
src/MultimediaSystem/thirdparty/agast/src/%.o: ../src/MultimediaSystem/thirdparty/agast/src/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I/home/alexis/BeagleBone/gsl-arm -I/home/alexis/BeagleBone/boost-arm -I/home/alexis/BeagleBone/gsl-arm/include -I/home/alexis/BeagleBone/lpsolve-arm -I/home/alexis/BeagleBone/opencv-arm/include -I/home/alexis/BeagleBone/opencv-arm -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/ASN.1" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/MultimediaSystem/includes" -O0 -g3 -Wall -c -fmessage-length=0 -g2 -mfpu=neon -mfloat-abi=hard -flax-vector-conversions -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


