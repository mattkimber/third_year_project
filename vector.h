/*
 * Third Year Project
 * Matt Kimber
 *
 * Vectors header
 */


/* Dot and cross products */
#define DOT(v1,v2) ((v1[0] * v2[0]) + (v1[1] * v2[1]) + (v1[2] * v2[2]));

#define CROSS(dest,v1,v2) \
              dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
              dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
              dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

/* Subtract two vectors */
#define SUBTRACT(dest,v1,v2) \
                 dest[0] = v1[0]-v2[0]; \
                 dest[1] = v1[1]-v2[1]; \
                 dest[2] = v1[2]-v2[2];

