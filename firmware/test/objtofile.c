#include <stdio.h>
#include <limits.h>
#include <string.h>

int main( int argc, char ** argv )
{
	if( argc < 2 )
	{
		fprintf( stderr, "objtofile [obj file]\n" );
		return -6;
	}

	FILE * f = fopen( argv[1], "r" );
	if( !f )
	{
		fprintf( stderr, "Error: can't open %s\n", argv[1] );
		return -5;
	}
	char buffer[1024];
	float scale = 26000;
	char * line;
	int mode = 1;

	char *s = strrchr( argv[1], '.' );
	if( s ) *s = 0;

	char outname[PATH_MAX];
	snprintf( outname, sizeof(outname)-1, "%s.h", argv[1] );

	FILE * bh = fopen( outname, "w" );
	fprintf( bh, "#include <stdint.h>\nint16_t %s_verts[] = {\n", argv[1] );
	while( line = fgets( buffer, sizeof(buffer)-1, f ) )
	{
		if( line[0] == 'v' )
		{
			float x, y, z;
			sscanf( line + 2, "%f %f %f", &x, &y, &z );
			fprintf( bh, "\t%d, %d, %d,\n", (int)(x*scale), (int)(y*scale), (int)(z*scale) );
		}
		if( line[0] == 'l' )
		{
			if( mode == 1 )
			{
				mode = 2;
				fprintf( bh, "};\nuint8_t %s_lines[] = {\n", argv[1] );
			}

			int x, y;
			sscanf( line + 2, "%d %d", &x, &y );
			fprintf( bh, "\t%d, %d,\n", x-1, y-1 );
		}
	}
	fprintf( bh, "};\n" );

}

