# Host unit tests for the portable firmware modules (no STM32 needed).
#   make            build + run tests
#   make check-bsp  syntax-check the STM32 BSP against HAL stubs
CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -Werror -O1 -g
APP      = ../App
SRC      = $(APP)/app.c $(APP)/communication/protocol.c $(APP)/safety/safety.c \
           $(APP)/motor/motor.c $(APP)/servo/servo.c
TESTS    = test_main.c test_protocol.c test_app.c fake_bsp.c
VECTORS  = ../../interface/test-vectors/protocol_v1.txt

all: run

fw_tests: $(SRC) $(TESTS) $(APP)/*.h $(APP)/*/*.h
	$(CC) $(CFLAGS) -o $@ $(SRC) $(TESTS)

run: fw_tests check-bsp
	./fw_tests $(VECTORS)

check-bsp:
	$(CC) -std=c99 -Wall -Wextra -Werror -fsyntax-only -DUSE_HAL_DRIVER -Istubs $(APP)/bsp/bsp_stm32h7.c

clean:
	rm -f fw_tests

.PHONY: all run check-bsp clean
