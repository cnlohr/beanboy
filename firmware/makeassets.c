#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <stdint.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

char ** fileList;
int nrFiles;

void enumerate( const char * dirName )
{
	DIR* dir = opendir( dirName );
	struct dirent *pent = NULL;
	if( !dir )
	{
		fprintf( stderr, "Error: Could not open folder %s\n", dirName );
		return;
	}
	while (pent = readdir (dir)) 
	{
		if ( pent->d_name[0] == '.' )
			continue;

		char * fileName = (char*)malloc( strlen(dirName) + strlen(pent->d_name) + 2 );
		strcpy( fileName, dirName );
		strcat( fileName, "/" );
		strcat( fileName, pent->d_name );

		struct stat st;
		stat(fileName, &st);

		if( S_ISDIR( st.st_mode ) )
		{
			enumerate( fileName );
		}
		else
		{
			fileList = realloc( fileList, (nrFiles + 1) * sizeof( char * ) );
			fileList[nrFiles++] = fileName;
		}
	}
}

int main()
{
	int i;

	FILE * fAssets = fopen( "assets.h", "w" );

	fprintf( fAssets, "#ifndef _ASSETS_H\n" );
	fprintf( fAssets, "#define _ASSETS_H\n" );
	fprintf( fAssets, "\n" );
	fprintf( fAssets, "#include <stdint.h>\n" );
	fprintf( fAssets, "\n" );
	fprintf( fAssets, "typedef struct {\n" );
	fprintf( fAssets, "\tint16_t w;\n" );
	fprintf( fAssets, "\tint16_t h;\n" );
	fprintf( fAssets, "\tconst uint32_t data[];\n" );
	fprintf( fAssets, "} bsprite;\n" );
	fprintf( fAssets, "\n" );

	const char * rootAssets = "assets";
	enumerate( rootAssets );

	qsort( fileList, nrFiles, sizeof(char*), (int (*)(const void*, const void*))strcmp );

	for( i = 0; i < nrFiles; i++ )
	{
		const char * fName = fileList[i];
		char * extension = strrchr( fName, '.' );
		char * base = strrchr( fName, '/' );
	
		if( !extension )
		{
			fprintf( stderr, "Error: No extension for file %s\n", fName );
			continue;
		}
		if( !base )
		{
			fprintf( stderr, "Error: No base for file %s\n", fName );
			continue;
		}

		int baseLen = (extension-base-1);
		if( baseLen <= 0 )
		{
			fprintf( stderr, "Error: Invalid file %s\n", fName );
			continue;
		}

		char baseName[baseLen+1];
		memcpy( baseName, base+1, baseLen );
		baseName[baseLen] = 0;

		if( strcmp( extension, ".png" ) == 0 )
		{
			printf( ".png extension\n" );
			int w,h,n;
			uint32_t * data = (uint32_t*)stbi_load( fName, &w, &h, &n, 4);

			int ow = (w + 15)&(~15);
			int oh = h;
			int x, y;
			uint32_t * eho = calloc( ow * oh / 16, 4 );
			for( y = 0; y < h; y++ )
			{
				for( x = 0; x < w; x++ )
				{
					uint32_t px = data[x+y*w];
					int r = px & 0xff;
					int g = (px>>8) & 0xff;
					int b = (px>>16) & 0xff;
					int a = (px>>24) & 0xff;

					if( a < 128 )
						; // Transparent
					else if( (((r + b)>>1) - g) > 200 ) // Purple
						; // Transparent
					else
					{
						uint32_t maskin = (((r+g+b)>383) << 16) | 0x1;
						maskin <<= (x&15);
						eho[(x+y*ow)>>4] |= maskin;
					}
				}
			}

			int i;
			fprintf( fAssets, "const bsprite %s   __attribute__((section(\".rodata\"))) = {\n", baseName );
			fprintf( fAssets, "\t.w = %d,\n", ow/16 );
			fprintf( fAssets, "\t.h = %d,\n", oh);
			fprintf( fAssets, "\t.data = {" );
			int words = ow*oh/16;
			for( i = 0; i < words; i++ )
			{
				if( (i & 0x7) == 0 ) fprintf( fAssets, "\n\t\t" );
				fprintf( fAssets, "0x%08x, ", eho[i] );
			}
			fprintf( fAssets, "\n\t}\n};\n" );
		}
	}

	fprintf( fAssets, "\n#endif\n\n" );

	return 0;
}



