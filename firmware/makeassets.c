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

	const char * rootAssets = "assets";
	enumerate( rootAssets );

	qsort( fileList, nrFiles, sizeof(char*), (__compar_fn_t)strcmp );

	for( i = 0; i < nrFiles; i++ )
	{
		const char * fName = fileList[i];
		char * extension = strrchr( fName, "." );
		if( !extension )
		{
			fprintf( stderr, "Error: No extension for file %s\n", fName );
			continue;
		}
		if( strcmp( extension, ".png" ) == 0 )
		{
			printf( ".png extension\n" );
			int w,h,n;
			unsigned char *data = stbi_load( fName, &w, &h, &n, 4);

			int ow = w;
			int oh = (h + 7)&(~7);
			int x, y;
			for( y = 0; y < h; y++ )
			{
				for( x = 0; x < w; x++ )
				{
				}
			}
		}
	}

	fprintf( fAssets, "\n#endif\n\n" );

	return 0;
}



