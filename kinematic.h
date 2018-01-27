#include "types.h"
#include <Arduino.h>
typedef struct joints_s joints_t;
typedef struct cart_s cart_t;
typedef struct vetor_s vetor_t; //定义三维向量类型
typedef struct matrix3d_s matrix3d_t; //定义三维矩阵类型
typedef struct homo_s homo_t;//定义其次矩阵类型
typedef struct dyn_s dyn_t;
const double a1 = 10.5, a2 = 104, a3 = 26, d1 = 105, d4 = 98, d6 = 85;  //DH_parameter
matrix3d_t eul2r(double a, double b, double c) ;
cart_t tr2eul(homo_t T);
joints_t m_ikine(homo_t h_matrix);
joints_t c_ikine(cart_t pos);
homo_t fkine(joints_t a);
void set_current_coord(joints_t a);
