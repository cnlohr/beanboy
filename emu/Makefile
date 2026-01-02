all : emu


CFLAGS:= -Os -s

emu : emu.c
	gcc $(CFLAGS) -o $@ $^ -lX11

# sudo apt-get install mingw-w64-x86-64-dev gcc-mingw-w64-x86-64
emu.exe : emu.c
	x86_64-w64-mingw32-gcc $(CFLAGS) -o $@ $^ -luser32 -lgdi32 -lws2_32 -liphlpapi

emu_win : emu.c
	gcc $(CFLAGS) -o emu.exe $^ -luser32 -lgdi32

clean :
	rm -rf emu emu.exe

