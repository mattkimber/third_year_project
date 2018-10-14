/* Bitmap functions */

/* Our own local header files */
#ifndef GRAPHICS
#include "graphics.h"
#endif

/* 24/32 bit, Targa file loader.  From the NeHe tutorials.  */
int LoadTGA(const char *filename, Image *img)
{
  FILE *file;

  GLubyte tga_header[12]={0,0,2,0,0,0,0,0,0,0,0,0};
  GLubyte tga_compare[12];
  GLubyte header[6];
  GLuint bytes_per_pixel;
  GLuint image_size;
  GLuint temp;

  int i;

  file = fopen(filename, "rb");

  /* Check we can read a valid header */
  if (file == NULL || 
      fread(tga_compare, 1, sizeof(tga_compare), file) != sizeof(tga_compare)
      || memcmp(tga_header, tga_compare, sizeof(tga_header)) != 0 ||
      fread(header,1,sizeof(header),file) != sizeof(header)) {
    if (file == NULL) {
      printf("Could not find targa file %s.\n", filename);
      return 0;
    } else {
      fclose(file);
      printf("Invalid header in targa file %s.\n", filename);
      return 0;
    }
  }

  /* Get the image size */
  img->width = header[1] * 256 + header[0];
  img->height = header[3] * 256 + header[2];
    
  /* Check this is a 24-bit or 32-bit image */
  if (img->width <= 0 || img->height <=0 || 
      (header[4]!=24 && header[4]!=32)) {
    fclose(file);
    printf("Targa file %s is not a 24 or 32 bit image.\n", filename);
    return 0;
  }

  /* Get the BPP */
  img->bpp = header[4];

  /* Figure out the image size */
  bytes_per_pixel = img->bpp/8;
  image_size = img->width * img->height * bytes_per_pixel;
  
  /* Allocate memory for the image and load the image into it */
  img->data = (GLubyte *)malloc(image_size);
  
  if (img->data == NULL || 
      fread(img->data, 1, image_size, file) != image_size) {
    if (img->data != NULL) free(img->data);
    fclose(file);
    return 0;
  }
  
  /* Swap colour data from BGR to RGB */
  for (i=0;i<image_size;i+=bytes_per_pixel) {
    temp = img->data[i];
    img->data[i] = img->data[i+2];
    img->data[i+2] = temp;
  }
  
  /* Close the file */
  fclose(file);
  return 1;


}

/* 24 bit, single plane bitmap loader.  From the NeHe tutorials Linux port. */
int LoadBitmap(const char *filename, Image *img) 
{

  FILE *file;
  unsigned long size; /* Size in bytes */
  unsigned long i; /* Loop counter */
  unsigned short int planes; /* Number of planes in the bitmap */
  

  char temp; /* Temporary colour storage for format conversion */

  /* Make sure the file exists */
  if ((file = fopen(filename, "rb"))==NULL) {
    printf("Bitmap file not found: %s\n",filename);
    return 0;
  }

  /* Get the width/height of the image from the header */
  fseek(file, 18, SEEK_CUR);

  /* Width */
  if ((i=fread(&img->width, 4, 1, file)) !=1) {
    printf("Error reading width from bitmap file %s.\n", filename);
    return 0;
  }

  /* Height */
  if ((i=fread(&img->height, 4, 1, file)) !=1) {
    printf("Error reading height from bitmap file %s.\n", filename);
    return 0;
  }  

  /* Calculate the size, assuming this is a 24bit image */
  size = img->width * img->height * 3;

  /* Read the number of planes to check we only have one */
  if ((fread(&planes, 2, 1, file)) != 1) {
    printf("Error reading number of planes from bitmap file %s.\n", filename);
    return 0;
  }

  if (planes != 1) {
    printf("Incorrect number of planes in bitmap file %s.\n", filename);
    return 0;
  }

  /* Check that we have the correct number of bits per pixel */
  if ((i=fread(&img->bpp, 2, 1, file)) != 1) {
    printf("Error reading colour depth from bitmap file %s.\n", filename);
    return 0;
  }

  if (img->bpp != 24) {
    printf("Incorrect colour depth in bitmap file %s.\n", filename);
    return 0;
  }

  /* Seek past the rest of the BMP header */
  fseek(file, 24, SEEK_CUR);

  /* Read the data */

  /* Try to allocate the memory for the image */
  img->data = (char *)malloc(size);

  if (img->data == NULL) {
    printf("Could not allocate memory in bitmap loader.\n");
    return 0;
  }

  /* Try to read the data from the file */
  if ((i=fread(img->data, size, 1, file)) != 1) {
    printf("Error reading bitmap data from bitmap file %s.\n", filename);
    return 0;
  }

  /* BMP is stored as BlueGreenRed, and we want RedGreenBlue, so
     we need to convert all of the colour triplets to the right
     way round */
  for(i=0;i<size;i+=3) {
    temp = img->data[i];
    img->data[i] = img->data[i+2];
    img->data[i+2] = temp;
  }

  /* Finished. */
  return 1;
}


