# beanboy

(Early drafts, not ready yet)

## Installation / getting started

Follow these instructions: ![https://github.com/cnlohr/ch32fun/wiki](https://github.com/cnlohr/ch32fun/wiki)



TODO:

1. Build up two beanboys
2. Check to see if the radio code works.
3. Make sprite import code.
4. Add transparency code.
5. Add API for comms.
6. "Pass the puffer"  "Hot Potato with puffer fish"
  * Choose between 2 and 6 players
  * Randomly assign X amount of passes.
  * # of passes remaining is unknown.
7. Shut the box.
  * Rules too complicated for me to write here.


Have .png files in `sprites/` folder, then on compile, they will turn into the object name itself.

TODO: Add transparency code.

```c
void drawSprite(int x, int y, const sprite_t * spr );
```

```c
// call these
void ISLERSetup( int channel ); 
void ISLERSend( uint8_t message, int messageLength );

struct MyGameStruct
{
	uint32_t id;
	uint8_t something;
} __attribute__((packed));

void ISLERCallback( uint8_t * message, int messageLength )
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

