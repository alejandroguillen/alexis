################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/ASN.1/ACKsliceMessage.c \
../src/ASN.1/AddCameraMessage.c \
../src/ASN.1/BIT_STRING.c \
../src/ASN.1/BOOLEAN.c \
../src/ASN.1/CodingChoices.c \
../src/ASN.1/CoopStatus.c \
../src/ASN.1/CooperatorInfo.c \
../src/ASN.1/Coordinate.c \
../src/ASN.1/DataATCMessage.c \
../src/ASN.1/DataCTAMessage.c \
../src/ASN.1/DescriptorTypes.c \
../src/ASN.1/DetectorTypes.c \
../src/ASN.1/INTEGER.c \
../src/ASN.1/NativeEnumerated.c \
../src/ASN.1/NativeInteger.c \
../src/ASN.1/NativeReal.c \
../src/ASN.1/OCTET_STRING.c \
../src/ASN.1/OffloadingChoices.c \
../src/ASN.1/REAL.c \
../src/ASN.1/StartATCMessage.c \
../src/ASN.1/StartCTAMessage.c \
../src/ASN.1/StartDATCMessage.c \
../src/ASN.1/asn_codecs_prim.c \
../src/ASN.1/ber_decoder.c \
../src/ASN.1/ber_tlv_length.c \
../src/ASN.1/ber_tlv_tag.c \
../src/ASN.1/constr_SEQUENCE.c \
../src/ASN.1/constr_TYPE.c \
../src/ASN.1/constraints.c \
../src/ASN.1/der_encoder.c \
../src/ASN.1/per_decoder.c \
../src/ASN.1/per_encoder.c \
../src/ASN.1/per_opentype.c \
../src/ASN.1/per_support.c \
../src/ASN.1/xer_decoder.c \
../src/ASN.1/xer_encoder.c \
../src/ASN.1/xer_support.c 

OBJS += \
./src/ASN.1/ACKsliceMessage.o \
./src/ASN.1/AddCameraMessage.o \
./src/ASN.1/BIT_STRING.o \
./src/ASN.1/BOOLEAN.o \
./src/ASN.1/CodingChoices.o \
./src/ASN.1/CoopStatus.o \
./src/ASN.1/CooperatorInfo.o \
./src/ASN.1/Coordinate.o \
./src/ASN.1/DataATCMessage.o \
./src/ASN.1/DataCTAMessage.o \
./src/ASN.1/DescriptorTypes.o \
./src/ASN.1/DetectorTypes.o \
./src/ASN.1/INTEGER.o \
./src/ASN.1/NativeEnumerated.o \
./src/ASN.1/NativeInteger.o \
./src/ASN.1/NativeReal.o \
./src/ASN.1/OCTET_STRING.o \
./src/ASN.1/OffloadingChoices.o \
./src/ASN.1/REAL.o \
./src/ASN.1/StartATCMessage.o \
./src/ASN.1/StartCTAMessage.o \
./src/ASN.1/StartDATCMessage.o \
./src/ASN.1/asn_codecs_prim.o \
./src/ASN.1/ber_decoder.o \
./src/ASN.1/ber_tlv_length.o \
./src/ASN.1/ber_tlv_tag.o \
./src/ASN.1/constr_SEQUENCE.o \
./src/ASN.1/constr_TYPE.o \
./src/ASN.1/constraints.o \
./src/ASN.1/der_encoder.o \
./src/ASN.1/per_decoder.o \
./src/ASN.1/per_encoder.o \
./src/ASN.1/per_opentype.o \
./src/ASN.1/per_support.o \
./src/ASN.1/xer_decoder.o \
./src/ASN.1/xer_encoder.o \
./src/ASN.1/xer_support.o 

C_DEPS += \
./src/ASN.1/ACKsliceMessage.d \
./src/ASN.1/AddCameraMessage.d \
./src/ASN.1/BIT_STRING.d \
./src/ASN.1/BOOLEAN.d \
./src/ASN.1/CodingChoices.d \
./src/ASN.1/CoopStatus.d \
./src/ASN.1/CooperatorInfo.d \
./src/ASN.1/Coordinate.d \
./src/ASN.1/DataATCMessage.d \
./src/ASN.1/DataCTAMessage.d \
./src/ASN.1/DescriptorTypes.d \
./src/ASN.1/DetectorTypes.d \
./src/ASN.1/INTEGER.d \
./src/ASN.1/NativeEnumerated.d \
./src/ASN.1/NativeInteger.d \
./src/ASN.1/NativeReal.d \
./src/ASN.1/OCTET_STRING.d \
./src/ASN.1/OffloadingChoices.d \
./src/ASN.1/REAL.d \
./src/ASN.1/StartATCMessage.d \
./src/ASN.1/StartCTAMessage.d \
./src/ASN.1/StartDATCMessage.d \
./src/ASN.1/asn_codecs_prim.d \
./src/ASN.1/ber_decoder.d \
./src/ASN.1/ber_tlv_length.d \
./src/ASN.1/ber_tlv_tag.d \
./src/ASN.1/constr_SEQUENCE.d \
./src/ASN.1/constr_TYPE.d \
./src/ASN.1/constraints.d \
./src/ASN.1/der_encoder.d \
./src/ASN.1/per_decoder.d \
./src/ASN.1/per_encoder.d \
./src/ASN.1/per_opentype.d \
./src/ASN.1/per_support.d \
./src/ASN.1/xer_decoder.d \
./src/ASN.1/xer_encoder.d \
./src/ASN.1/xer_support.d 


# Each subdirectory must supply rules for building sources it contributes
src/ASN.1/%.o: ../src/ASN.1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/greeneyes/OpenCV-Install/2.4.4/include -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/ASN.1" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src" -I"/home/alexis/Dropbox/THESIS/Testbed_code/workspace/jordi/src/MultimediaSystem/includes" -I/usr/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


