# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Antonio Niño Díaz, 2024

BLOCKSDS	?= /opt/blocksds/core
BLOCKSDSEXT	?= /opt/blocksds/external

# User config

NAME		:= b2_hello_world
GAME_TITLE	:= Hello World
GAME_TITLE	:= Box2D

# Defines passed to all files

DEFINES		:= -DTARGET_FLOAT32_IS_FIXED -DTARGET_IS_NDS

# Libraries

LIBS		:= -lbox2d -lnds9
LIBDIRS		:= $(BLOCKSDS)/libs/libnds \
		   $(BLOCKSDSEXT)/box2d

include $(BLOCKSDS)/sys/default_makefiles/rom_arm9/Makefile
