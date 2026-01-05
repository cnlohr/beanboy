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

	int minx = INT_MAX;
	int miny = INT_MAX;
	int minz = INT_MAX;
	int maxx = INT_MIN;
	int maxy = INT_MIN;
	int maxz = INT_MIN;

	fprintf( bh, "#include <stdint.h>\nconst int16_t %s_verts[]  __attribute__((section(\".rodata\"))) = {\n", argv[1] );
	while( line = fgets( buffer, sizeof(buffer)-1, f ) )
	{
		if( line[0] == 'v' )
		{
			float x, y, z;
			sscanf( line + 2, "%f %f %f", &x, &y, &z );
			int ox = (int)(x*scale), oy = (int)(y*scale), oz = (int)(z*scale);
			fprintf( bh, "\t%d, %d, %d,\n", ox, oy, oz );
			if( ox < minx ) minx = ox;
			if( oy < miny ) miny = oy;
			if( oz < minz ) minz = oz;
			if( ox > maxx ) maxx = ox;
			if( oy > maxy ) maxy = oy;
			if( oz > maxz ) maxz = oz;
		}
		if( line[0] == 'l' )
		{
			if( mode == 1 )
			{
				mode = 2;
				fprintf( bh, "};\nconst uint8_t %s_lines[]  __attribute__((section(\".rodata\"))) = {\n", argv[1] );
			}

			int x, y;
			sscanf( line + 2, "%d %d", &x, &y );
			fprintf( bh, "\t%d, %d,\n", x-1, y-1 );
		}
	}
	fprintf( bh, "};\n" );
	printf( "Extents: %d %d %d - %d %d %d\n", minx, miny, minz, maxx, maxy, maxz );

}

