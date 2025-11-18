#pragma once
struct Matrix3x3 {
   float m[3][3];

   Matrix3x3 operator+(const Matrix3x3& a) const {
	  Matrix3x3 result;
	  for (int i = 0; i < 3; i++) {
		 for (int j = 0; j < 3; j++) {
			result.m[i][j] = m[i][j] + a.m[i][j];
		 }
	  }
	  return result;
   }

   Matrix3x3 operator-(const Matrix3x3& a) const {
	  Matrix3x3 result;
	  for (int i = 0; i < 3; i++) {
		 for (int j = 0; j < 3; j++) {
			result.m[i][j] = m[i][j] - a.m[i][j];
		 }
	  }
	  return result;
   }

   Matrix3x3 operator*(const Matrix3x3& a) const {
	  Matrix3x3 result;
	  for (int i = 0; i < 4; i++) {
		 for (int j = 0; j < 4; j++) {
			result.m[i][j] = 0;
			for (int k = 0; k < 4; k++) {
			   result.m[i][j] += m[i][k] * a.m[k][j];
			}
		 }
	  }
	  return result;
   }
};