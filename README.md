# beanboy

(Early drafts, not ready yet)

## Installation / getting started

Follow these instructions: [https://github.com/cnlohr/ch32fun/wiki](https://github.com/cnlohr/ch32fun/wiki)

## Graphics API

The bean boy uses a monochrome 128x128 display.

### Input

You get "pressures" for button 1, button 2, button 3, and the battery level. (For now, just print the values to the screen, we will be changing the levels)

```c
	uint32_t pressures[4];
	BeanBoyReadPressures( pressures );
```

### Sprites

Have .png files in `sprites/` folder, then on compile, they will turn into the object name itself, when you say `make clean firmware.elf`

```c
void RenderBSprite( const bsprite * spr, int outx, int outy );
```

For instance if you have `sprites/bubble.png`, you can call:

```c
	RenderBSprite( &bubble, x, y );
```

You can use purple or actual transparency to indicate transparent, then use white or black.

### Basic Drawing

Graphics APIs are from [the ch32v003 ssd1306 driver](https://github.com/cnlohr/ch32fun/blob/master/extralibs/ssd1306.h)

```c
#define SSD1306_W 128
#define SSD1306_H 128


void ssd1306_drawPixel(uint32_t x, uint32_t y, int color);
void ssd1306_xorPixel(uint32_t x, uint32_t y);
void ssd1306_drawImage(uint32_t x, uint32_t y, const unsigned char* input, uint32_t width, uint32_t height, uint32_t color_mode);
void ssd1306_drawFastVLine(int32_t x, int32_t y, int32_t h, uint32_t color);
void ssd1306_drawFastHLine(uint32_t x, uint32_t y, uint32_t w, uint32_t color);
void ssd1306_drawLine(int x0, int y0, int x1, int y1, uint32_t color);
void ssd1306_drawCircle(int x, int y, int radius, int color);
void ssd1306_fillCircle(int x, int y, int radius, int color);
void ssd1306_drawRect(int32_t x, int32_t y, uint32_t w, uint32_t h, uint32_t color);
void ssd1306_fillRect(uint32_t x, uint32_t y, uint8_t w, uint32_t h, uint32_t color);
void ssd1306_xorrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void ssd1306_drawchar(uint8_t x, uint8_t y, uint8_t chr, uint8_t color);
void ssd1306_drawstr(uint8_t x, uint8_t y, char *str, uint8_t color);
void ssd1306_drawchar_sz(uint8_t x, uint8_t y, uint8_t chr, uint8_t color, font_size_t font_size);
void ssd1306_drawstr_sz(uint8_t x, uint8_t y, char *str, uint8_t color, font_size_t font_size);
```

### Wireless API

```c
// call these
void ISLERSetup( int channel ); 
void ISLERSend( uint8_t message, int messageLength );

struct MyGameStruct
{
	uint32_t id;
	uint8_t something;
} __attribute__((packed));

void ISLERCallback( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	struct MyGameStruct * st = (struct MyGameStruct*)message;

	if( messageLength < sizeof( struct MyGameStruct ) )
		return;

	printf( "%d\n", st->something );
}

void main()
{
	ISLERSetup( 9 );

	struct MyGameStruct st;
	st.id = 0x1234778a; // Some ID so you don't get confused with other BLE-messages.
	st.something = 5;

	ISLERSend( &st, sizeof( st ) );
}
```

## TODO


TODO:

1. ~~Build up two beanboys~~
2. ~~Check to see if the radio code works.~~
3. ~~Make sprite import code.~~
4. ~~Add transparency code.~~
5. ~~Add API for comms
6. "Pass the puffer"  "Hot Potato with puffer fish"
  * Choose between 2 and 6 players
  * Randomly assign X amount of passes.
  * # of passes remaining is unknown.
7. Shut the box.
  * Rules too complicated for me to write here.




