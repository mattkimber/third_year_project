/*
 * 3rd year project
 * Matt Kimber
 *
 * Matrix manipulation functions
 */

/* ODE headers needed for the dMatrix3 type */
#include <ode/ode.h>

void MatrixMultiply(dMatrix3 A, dMatrix3 B, dMatrix3 dest)
{
  /* Multiply the two matrices and place the result in dest */
  dest[0] = (A[0]*B[0]) + (A[1]*B[4]) + (A[2] * B[8]);
  dest[1] = (A[0]*B[1]) + (A[1]*B[5]) + (A[2] * B[9]);
  dest[2] = (A[0]*B[2]) + (A[1]*B[6]) + (A[2] * B[10]);

  dest[4] = (A[4]*B[0]) + (A[5]*B[4]) + (A[6] * B[8]);
  dest[5] = (A[4]*B[1]) + (A[5]*B[5]) + (A[6] * B[9]);
  dest[6] = (A[4]*B[2]) + (A[5]*B[6]) + (A[6] * B[10]);

  dest[8] = (A[8]*B[0]) + (A[9]*B[4]) + (A[10] * B[8]);
  dest[9] = (A[8]*B[1]) + (A[9]*B[5]) + (A[10] * B[9]);
  dest[10] = (A[8]*B[2]) + (A[9]*B[6]) + (A[10] * B[10]);
}

void MatrixTranspose(dMatrix3 A, dMatrix3 dest)
{

  dest[0] = A[0];
  dest[1] = A[4];
  dest[2] = A[8];

  dest[4] = A[1];
  dest[5] = A[5];
  dest[6] = A[9];

  dest[8] = A[2];
  dest[9] = A[6];
  dest[10] = A[10];


}



void GenerateRotationMatrix(dMatrix3 dest, float rx, float ry, float rz)
{
  dMatrix3 r[3];
  dMatrix3 t;

  dRFromAxisAndAngle(r[0], 1.0, 0.0, 0.0, rx);
  dRFromAxisAndAngle(r[1], 0.0, 1.0, 0.0, ry);
  dRFromAxisAndAngle(r[2], 0.0, 0.0, 1.0, rz);

  /* Reset the rotation matrix */
  MatrixMultiply(r[0], r[1], t);
  MatrixMultiply(t, r[2], dest);
}
