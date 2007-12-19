/*
** Easylogo TGA->header converter
** ==============================
** (C) 2000 by Paolo Scaffardi (arsenio@tin.it)
** AIRVENT SAM s.p.a - RIMINI(ITALY)
**
** This is still under construction!
*/

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#pragma pack(1)

/*#define ENABLE_ASCII_BANNERS */

typedef struct {
	unsigned char	id;
	unsigned char	ColorMapType;
	unsigned char	ImageTypeCode;
	unsigned short	ColorMapOrigin;
	unsigned short	ColorMapLenght;
	unsigned char	ColorMapEntrySize;
	unsigned short	ImageXOrigin;
	unsigned short	ImageYOrigin;
	unsigned short	ImageWidth;
	unsigned short	ImageHeight;
	unsigned char	ImagePixelSize;
	unsigned char	ImageDescriptorByte;
} tga_header_t;

typedef struct {
	unsigned char r,g,b ;
} rgb_t ;

typedef struct {
	unsigned char b,g,r ;
} bgr_t ;

typedef struct {
	unsigned char 	Cb,y1,Cr,y2;
} yuyv_t ;

typedef struct {
	void				*data,
					*palette ;
	int				width,
					height,
					pixels,
					bpp,
					pixel_size,
					size,
					palette_size,
					yuyv;
} image_t ;

void StringUpperCase (char *str)
{
    int count = strlen(str);
    char c ;

    while(count--)
    {
	c=*str;
	if ((c >= 'a')&&(c<='z'))
	    *str = 'A' + (c-'a');
	str++ ;
    }
}

void StringLowerCase (char *str)
{
    int count = strlen(str);
    char c ;

    while(count--)
    {
	c=*str;
	if ((c >= 'A')&&(c<='Z'))
	    *str = 'a' + (c-'A');
	str++ ;
    }
}
void pixel_rgb_to_yuyv (rgb_t *rgb_pixel, yuyv_t *yuyv_pixel)
{
    unsigned int pR, pG, pB ;

    /* Transform (0-255) components to (0-100) */
    pR = rgb_pixel->r * 100 / 255 ;
    pG = rgb_pixel->g * 100 / 255 ;
    pB = rgb_pixel->b * 100 / 255 ;

    /* Calculate YUV values (0-255) from RGB beetween 0-100 */
    yuyv_pixel->y1 = yuyv_pixel->y2 	= 209 * (pR + pG + pB) / 300 + 16  ;
    yuyv_pixel->Cb 			= pB - (pR/4)   - (pG*3/4)   + 128 ;
    yuyv_pixel->Cr 			= pR - (pG*3/4) - (pB/4)     + 128 ;

    return ;
}

void printlogo_rgb (rgb_t	*data, int w, int h)
{
    int x,y;
    for (y=0; y<h; y++)
    {
	for (x=0; x<w; x++, data++)
	    if ((data->r < 30)/*&&(data->g == 0)&&(data->b == 0)*/)
		printf(" ");
	    else
		printf("X");
	printf("\n");
    }
}

void printlogo_yuyv (unsigned short *data, int w, int h)
{
    int x,y;
    for (y=0; y<h; y++)
    {
	for (x=0; x<w; x++, data++)
	    if (*data == 0x1080)    /* Because of inverted on i386! */
		printf(" ");
	    else
		printf("X");
	printf("\n");
    }
}

static inline unsigned short le16_to_cpu (unsigned short val)
{
    union {
	unsigned char pval[2];
	unsigned short val;
    } swapped;
    swapped.val = val;
    return (swapped.pval[1] << 8) + swapped.pval[0];
}

int image_load_tga (image_t *image, char *filename)
{
    FILE *file ;
    tga_header_t header ;
    int i;
    unsigned char app ;
    rgb_t *p ;

    if( ( file = fopen( filename, "rb" ) ) == NULL )
	return -1;

    fread(&header, sizeof(header), 1, file);

    /* byte swap: tga is little endian, host is ??? */
    header.ColorMapOrigin = le16_to_cpu (header.ColorMapOrigin);
    header.ColorMapLenght = le16_to_cpu (header.ColorMapLenght);
    header.ImageXOrigin = le16_to_cpu (header.ImageXOrigin);
    header.ImageYOrigin = le16_to_cpu (header.ImageYOrigin);
    header.ImageWidth = le16_to_cpu (header.ImageWidth);
    header.ImageHeight = le16_to_cpu (header.ImageHeight);

    image->width 	= header.ImageWidth ;
    image->height 	= header.ImageHeight ;

    switch (header.ImageTypeCode){
	case 2:	/* Uncompressed RGB */
			image->yuyv = 0 ;
			image->palette_size = 0 ;
			image->palette = NULL ;
	    break;

	default:
	    printf("Format not supported!\n");
	    return -1 ;
    }

    image->bpp  		= header.ImagePixelSize ;
    image->pixel_size 		= ((image->bpp-1) / 8) + 1 ;
    image->pixels 		= image->width * image->height;
    image->size 		= image->pixels * image->pixel_size ;
    image->data 		= malloc(image->size) ;

    if (image->bpp != 24)
    {
	printf("Bpp not supported: %d!\n", image->bpp);
	return -1 ;
    }

    fread(image->data, image->size, 1, file);

/* Swapping R and B values */

    p = image->data ;
    for(i=0; i < image->pixels; i++, p++)
    {
	app = p->r ;
	p->r = p->b ;
	p->b = app ;
    }

/* Swapping image */

    if(!(header.ImageDescriptorByte & 0x20))
    {
	unsigned char *temp = malloc(image->size);
	int linesize = image->pixel_size * image->width ;
	void	*dest = image->data,
		*source = temp + image->size - linesize ;

	printf("S");
	if (temp == NULL)
	{
	    printf("Cannot alloc temp buffer!\n");
	    return -1;
	}

	memcpy(temp, image->data, image->size);
	for(i = 0; i<image->height; i++, dest+=linesize, source-=linesize)
	    memcpy(dest, source, linesize);

	free( temp );
    }

#ifdef ENABLE_ASCII_BANNERS
    printlogo_rgb (image->data,image->width, image->height);
#endif

    fclose (file);
    return 0;
}

int image_free (image_t *image)
{
    if(image->data != NULL)
		free(image->data);

    if(image->palette != NULL)
		free(image->palette);

	return 0;
}

int image_rgb_to_yuyv (image_t *rgb_image, image_t *yuyv_image)
{
	rgb_t	*rgb_ptr = (rgb_t *) rgb_image->data ;
	yuyv_t	yuyv ;
	unsigned short *dest ;
	int	count = 0 ;

	yuyv_image->pixel_size 		= 2 ;
	yuyv_image->bpp			= 16 ;
	yuyv_image->yuyv		= 1 ;
	yuyv_image->width		= rgb_image->width ;
	yuyv_image->height		= rgb_image->height ;
	yuyv_image->pixels 		= yuyv_image->width * yuyv_image->height ;
	yuyv_image->size 		= yuyv_image->pixels * yuyv_image->pixel_size ;
	dest = (unsigned short *) (yuyv_image->data	= malloc(yuyv_image->size)) ;
	yuyv_image->palette		= 0 ;
	yuyv_image->palette_size= 0 ;

	while((count++) < rgb_image->pixels)
	{
		pixel_rgb_to_yuyv (rgb_ptr++, &yuyv);

		if ((count & 1)==0)	/* Was == 0 */
		    memcpy (dest, ((void *)&yuyv) + 2, sizeof(short));
		else
		    memcpy (dest, (void *)&yuyv, sizeof(short));

		dest ++ ;
	}

#ifdef ENABLE_ASCII_BANNERS
	printlogo_yuyv (yuyv_image->data, yuyv_image->width, yuyv_image->height);
#endif
	return 0 ;
}

#ifdef ENABLE_GZIP
int use_gzip = 0;
#endif

int image_save_header (image_t *image, char *filename, char *varname)
{
	FILE    *file = fopen (filename, "w");
	char	app[256], str[256]="", def_name[64] ;
	int 	count = image->size, col=0;
	unsigned char *dataptr = image->data ;
	if (file==NULL)
		return -1 ;

/*  Author information */
	fprintf(file, "/*\n * Generated by EasyLogo, (C) 2000 by Paolo Scaffardi\n *\n");
	fprintf(file, " * To use this, include it and call: easylogo_plot(screen,&%s, width,x,y)\n *\n", varname);
	fprintf(file, " * Where:\t'screen'\tis the pointer to the frame buffer\n");
	fprintf(file, " *\t\t'width'\tis the screen width\n");
	fprintf(file, " *\t\t'x'\t\tis the horizontal position\n");
	fprintf(file, " *\t\t'y'\t\tis the vertical position\n */\n\n");

/*  gzip compress */
	if (use_gzip) {
		unsigned char *compressed;
		struct stat st;
		FILE *gz;

		sprintf(str, "%s.gz", filename);
		sprintf(app, "gzip > %s", str);
		gz = popen(app, "w");
		fwrite(image->data, image->size, 1, gz);
		pclose(gz);

		gz = fopen(str, "r");
		stat(str, &st);
		compressed = malloc(st.st_size);
		fread(compressed, st.st_size, 1, gz);
		fclose(gz);

		unlink(str);

		dataptr = compressed;
		count = st.st_size;
		fprintf(file, "#define EASYLOGO_ENABLE_GZIP %i\n\n", count);
	}

/*	Headers */
	fprintf(file, "#include <video_easylogo.h>\n\n");
/*	Macros */
	strcpy(def_name, varname);
	StringUpperCase (def_name);
	fprintf(file, "#define	DEF_%s_WIDTH\t\t%d\n", def_name, image->width);
	fprintf(file, "#define	DEF_%s_HEIGHT\t\t%d\n", def_name, image->height);
	fprintf(file, "#define	DEF_%s_PIXELS\t\t%d\n", def_name, image->pixels);
	fprintf(file, "#define	DEF_%s_BPP\t\t%d\n", def_name, image->bpp);
	fprintf(file, "#define	DEF_%s_PIXEL_SIZE\t%d\n", def_name, image->pixel_size);
	fprintf(file, "#define	DEF_%s_SIZE\t\t%d\n\n", def_name, image->size);
/*  Declaration */
	fprintf(file, "unsigned char DEF_%s_DATA[] = {\n", def_name);

/*	Data */
	while(count)
		switch (col){
			case 0:
				sprintf(str, " 0x%02x", *dataptr++);
				col++;
				count-- ;
				break;

			case 16:
				fprintf(file, "%s", str);
				if (count > 0)
				    fprintf(file,",");
				fprintf(file, "\n");

				col = 0 ;
				break;

			default:
				strcpy(app, str);
				sprintf(str, "%s, 0x%02x", app, *dataptr++);
				col++ ;
				count-- ;
				break;
		}

	if (col)
		fprintf(file, "%s\n", str);

/* 	End of declaration */
	fprintf(file, "};\n\n");
/*	Variable */
	fprintf(file, "fastimage_t %s = {\n", varname);
	fprintf(file, "		DEF_%s_DATA,\n", def_name);
	fprintf(file, "		DEF_%s_WIDTH,\n", def_name);
	fprintf(file, "		DEF_%s_HEIGHT,\n", def_name);
	fprintf(file, "		DEF_%s_BPP,\n", def_name);
	fprintf(file, "		DEF_%s_PIXEL_SIZE,\n", def_name);
	fprintf(file, "		DEF_%s_SIZE\n};\n", def_name);

	fclose (file);

	return 0 ;
}

#define DEF_FILELEN	256

static void usage(int exit_status)
{
	printf(
		"EasyLogo 1.0 (C) 2000 by Paolo Scaffardi\n"
		"\n"
		"Syntax:	easylogo [options] inputfile [outputvar [outputfile]]\n"
		"\n"
		"Options:\n"
		"  -r     Output 24-bit RGB instead of yuyv\n"
#ifdef ENABLE_GZIP
		"  -g     Compress with gzip\n"
#endif
		"  -h     Help output\n"
		"\n"
		"Where: 'inputfile'   is the TGA image to load\n"
		"       'outputvar'   is the variable name to create\n"
		"       'outputfile'  is the output header file (default is 'inputfile.h')\n"
	);
	exit(exit_status);
}

int main (int argc, char *argv[])
{
    int c, use_rgb = 0;
    char
	inputfile[DEF_FILELEN],
	outputfile[DEF_FILELEN],
	varname[DEF_FILELEN];

    image_t 		rgb_logo, yuyv_logo ;

	while ((c = getopt(argc, argv, "hrg")) > 0) {
		switch (c) {
		case 'h':
			usage(0);
			break;
		case 'r':
			use_rgb = 1;
			printf("Using 24-bit RGB Output Fromat\n");
			break;
#ifdef ENABLE_GZIP
		case 'g':
			use_gzip = 1;
			puts("Compressing with gzip");
			break;
#endif
		default:
			usage(1);
			break;
		}
	}

	c = argc - optind;
	if (c > 4 || c < 1)
		usage(1);

	strcpy(inputfile, argv[optind]);

	if (c > 1)
		strcpy(varname, argv[optind + 1]);
	else {
		char *dot;
		strcpy(varname, inputfile);
		dot = strchr(varname, '.');
		if (dot)
			*dot = '\0';
	}

	if (c > 2)
		strcpy(outputfile, argv[optind + 2]);
	else {
		char *dot;
		strcpy(outputfile, inputfile);
		dot = strchr(outputfile, '.');
		if (dot)
			*dot = '\0';
	}

    setbuf(stdout, NULL);

    printf("Doing '%s' (%s) from '%s'...",
	outputfile, varname, inputfile);

/* Import TGA logo */

    printf("L");
    if (image_load_tga (&rgb_logo, inputfile)<0)
    {
	printf("input file not found!\n");
	exit(1);
    }

/* Convert it to YUYV format */

	if (!use_rgb) {
	    printf("C");
	    image_rgb_to_yuyv(&rgb_logo, &yuyv_logo) ;
	}
/* Save it into a header format */

    printf("S");
	if (use_rgb)
	    image_save_header(&rgb_logo, outputfile, varname);
	else
	    image_save_header(&yuyv_logo, outputfile, varname);
/* Free original image and copy */

    image_free (&rgb_logo);

	if (!use_rgb)
	    image_free(&yuyv_logo);

    printf("\n");

    return 0 ;
}
