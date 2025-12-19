all : flash

TARGET:=firmware
TARGET_MCU:=CH572
TARGET_MCU_PACKAGE:=CH572Q

EXTRA_ELF_DEPENDENCIES=assets.h

include ../ch32fun/ch32fun/ch32fun.mk

assets.h : makeassets
	./makeassets

makeassets : makeassets.c
	gcc -g -o $@ $^ -lm

clean :
	rm -rf *.o assets.h

clobber : clean
	makeassets

flash : cv_flash
clean : cv_clean
