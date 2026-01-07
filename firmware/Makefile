all : flash

TARGET:=firmware
TARGET_MCU:=CH572
TARGET_MCU_PACKAGE:=CH572Q
LINKER_SCRIPT:=linkerfile.ld
EXTRA_CFLAGS:=-Wno-unused-variable -Wno-unused-but-set-variable -pedantic -Wall
EXTRA_ELF_DEPENDENCIES=assets.h

include ../ch32fun/ch32fun/ch32fun.mk

assets.h : makeassets
	./makeassets

makeassets : makeassets.c
	gcc -g -o $@ $^ -lm

makeassets.exe : makeassets.c
	x86_64-w64-mingw32-gcc -flto -s -Os -o $@ $^

clean :
	rm -rf *.o assets.h

clobber : clean
	rm -rf makeassets makeassets.exe

flash : cv_flash
clean : cv_clean
