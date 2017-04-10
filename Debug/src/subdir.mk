################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/_write.c \
../src/ade.c \
../src/buffer.c \
../src/can.c \
../src/main.c \
../src/spi.c \
../src/stm32f10x_it.c \
../src/time.c 

OBJS += \
./src/_write.o \
./src/ade.o \
./src/buffer.o \
./src/can.o \
./src/main.o \
./src/spi.o \
./src/stm32f10x_it.o \
./src/time.o 

C_DEPS += \
./src/_write.d \
./src/ade.d \
./src/buffer.d \
./src/can.d \
./src/main.d \
./src/spi.d \
./src/stm32f10x_it.d \
./src/time.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSYSCLK_FREQ_48MHz=48000000 -DADE7953 -I"/home/jet/workspace/count.spi.f1/inc" -I"/home/jet/workspace/count.spi.f1/src" -I"/home/jet/workspace/count.spi.f1/system/include" -I"/home/jet/workspace/count.spi.f1/system/include/cmsis" -I"/home/jet/workspace/count.spi.f1/system/include/stm32f1-stdperiph" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


