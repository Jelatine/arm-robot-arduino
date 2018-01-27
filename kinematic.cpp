#include "kinematic.h"
extern joints_t cur_joint;
extern cart_t cur_cart;
extern homo_t cur_homo;

//欧拉角转旋转矩阵
matrix3d_t eul2r(double a, double b, double c) { //matlab eul2r('xyz')
  a = a * PI / 180; b = b * PI / 180; c = c * PI / 180;
  matrix3d_t matrix;
  matrix.n.vx = cos(a) * cos(b) * cos(c) - sin(a) * sin(c);
  matrix.n.vy = cos(a) * sin(c) + cos(b) * cos(c) * sin(a);
  matrix.n.vz = -cos(c) * sin(b);
  matrix.o.vx = - cos(c) * sin(a) - cos(a) * cos(b) * sin(c);
  matrix.o.vy = cos(a) * cos(c) - cos(b) * sin(a) * sin(c);
  matrix.o.vz = sin(b) * sin(c);
  matrix.a.vx = cos(a) * sin(b);
  matrix.a.vy = sin(a) * sin(b);
  matrix.a.vz = cos(b);
  //  matrix.n.vx = cos(a) * cos(b) * cos(c) - sin(a) * sin(c);
  //  matrix.o.vx = -cos(a) * cos(b) * sin(c) - sin(a) * cos(c);
  //  matrix.a.vx = cos(a) * sin(b);
  //  matrix.n.vy = sin(a) * cos(b) * cos(c) + cos(a) * sin(c);
  //  matrix.o.vy = -sin(a) * cos(b) * sin(c) + cos(a) * cos(c);
  //  matrix.a.vy = sin(a) * sin(b);
  //  matrix.n.vz = -sin(b) * cos(c);
  //  matrix.o.vz = sin(b) * sin(c);
  //  matrix.a.vz = cos(b);
  return matrix;
}
//齐次矩阵转欧拉角
cart_t tr2eul(homo_t T) {
  cart_t cart;
  cart.x = T.x; cart.y = T.y; cart.z = T.z;
  double sp, cp,e1;
  double eps = 0.001;
  if (fabs(T.r_matrix.a.vx) < eps && fabs(T.r_matrix.a.vy) < eps) {
    //singularity
    cart.a = 0;
    sp = 0;
    cp = 1;
    cart.b = atan2(cp * T.r_matrix.a.vx + sp * T.r_matrix.a.vy, T.r_matrix.a.vz) * 180 / PI;
    cart.c = atan2(-sp * T.r_matrix.n.vx + cp * T.r_matrix.n.vy, -sp * T.r_matrix.o.vx + cp * T.r_matrix.o.vy) * 180 / PI;
  }
  else {
    //non singular  Only positive phi is returned.
    //eul(1) = atan2(-R(2,3), -R(1,3));
    e1=atan2(T.r_matrix.a.vy, T.r_matrix.a.vx);
    cart.a = e1 * 180 / PI;
    sp = sin(e1);
    cp = cos(e1);
    cart.b = atan2(cp * T.r_matrix.a.vx + sp * T.r_matrix.a.vy, T.r_matrix.a.vz) * 180 / PI;
    cart.c = atan2(-sp * T.r_matrix.n.vx + cp * T.r_matrix.n.vy, -sp * T.r_matrix.o.vx + cp * T.r_matrix.o.vy) * 180 / PI;
  }
  return cart;
}

//输入齐次矩阵，求运动学逆解
joints_t m_ikine(homo_t h_matrix) {
  double joints[6];
  matrix3d_t matrix3d = h_matrix.r_matrix;
  double n11 = matrix3d.n.vx, n12 = matrix3d.o.vx, n13 = matrix3d.a.vx, n14 = h_matrix.x;
  double n21 = matrix3d.n.vy, n22 = matrix3d.o.vy, n23 = matrix3d.a.vy, n24 = h_matrix.y;
  double n31 = matrix3d.n.vz, n32 = matrix3d.o.vz, n33 = matrix3d.a.vz, n34 = h_matrix.z;
  double p1, p2;
  p1 = n24 - d6 * n23; p2 = n14 - d6 * n13;
  double q1 = atan2(p1, p2);  //Q1
  double c1, c2, c3;
  c1 = n14 * cos(q1) + n24 * sin(q1) - d6 * (n13 * cos(q1) + n23 * sin(q1)) - a1;
  c2 = n34 - d6 * n33 - d1;
  c3 = (c1 * c1 + c2 * c2 - d4 * d4 + a2 * a2 - a3 * a3) / (2 * a2);
  p1 = c3 / sqrt(c1 * c1 + c2 * c2);
  p2 = sqrt(1 - (c3 * c3) / (c1 * c1 + c2 * c2));
  double q2 = atan2(p1, p2) - atan2(c2, c1); //Q2
  p1 = (c3 - a2) / sqrt(d4 * d4 + a3 * a3);
  p2 = sqrt(1 - (c3 - a2) * (c3 - a2) / (d4 * d4 + a3 * a3));
  double q3 = atan2(p1, -p2) - atan2(a3, -d4); //Q3
  if (q3 < -PI)
    q3 = q3 + 2 * PI;
  p1 = n13 * sin(q1) - n23 * cos(q1);
  p2 = n33 * cos(q2 + q3) + sin(q2 + q3) * (n13 * cos(q1) + n23 * sin(q1));
  double q4 = atan2(p1, p2);
  double q5 = -acos(cos(q2 + q3) * (n13 * cos(q1) + n23 * sin(q1)) - n33 * sin(q2 + q3));
  p1 = cos(q2 + q3) * (n12 * cos(q1) + n22 * sin(q1)) - n32 * sin(q2 + q3);
  p2 = cos(q2 + q3) * (n11 * cos(q1) + n21 * sin(q1)) - n31 * sin(q2 + q3);
  double q6 = atan2(p1, -p2);
  //Serial.print("q4:=");Serial.println(q4);
  if (q4 > PI / 2 || q4 < -PI / 2)
  {

    p1 = n13 * sin(q1) - n23 * cos(q1);
    p2 = n33 * cos(q2 + q3) + sin(q2 + q3) * (n13 * cos(q1) + n23 * sin(q1));
    q4 = atan2(-p1, -p2);
    q5 = acos(cos(q2 + q3) * (n13 * cos(q1) + n23 * sin(q1)) - n33 * sin(q2 + q3));
    p1 = cos(q2 + q3) * (n12 * cos(q1) + n22 * sin(q1)) - n32 * sin(q2 + q3);
    p2 = cos(q2 + q3) * (n11 * cos(q1) + n21 * sin(q1)) - n31 * sin(q2 + q3);
    q6 = atan2(-p1, p2);
    //Serial.print("x q4:=");Serial.println(q4);
  }
  joints[0] = q1 * 180 / PI; joints[1] = q2 * 180 / PI; joints[2] = q3 * 180 / PI; joints[3] = q4 * 180 / PI; joints[4] = q5 * 180 / PI; joints[5] = q6 * 180 / PI;
  joints_t rt;// = {joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]};
  rt.q1 = joints[0]; rt.q2 = joints[1]; rt.q3 = joints[2]; rt.q4 = joints[3]; rt.q5 = joints[4]; rt.q6 = joints[5];
  //Serial.print("Q:=");Serial.print(rt.q1);Serial.print(rt.q2);Serial.print(rt.q3);Serial.print(rt.q4);Serial.print(rt.q5);Serial.println(rt.q6);
  return rt;
}

//输入笛卡尔坐标，求运动学逆解
joints_t c_ikine(cart_t pos) {
  homo_t homo;
  homo.x = pos.x; homo.y = pos.y; homo.z = pos.z;
  homo.r_matrix = eul2r(pos.a, pos.b, pos.c);
  joints_t rt = m_ikine(homo);
  return rt;
}

//运动学正解（关节->矩阵）
homo_t fkine(joints_t a) {
  double q1, q2, q3, q4, q5, q6;
  q1 = a.q1 * PI / 180; q2 = a.q2 * PI / 180; q3 = a.q3 * PI / 180; q4 = a.q4 * PI / 180; q5 = a.q5 * PI / 180; q6 = a.q6 * PI / 180;
  homo_t rt;
  rt.x = a1 * cos(q1) - d6 * (sin(q5) * (sin(q1) * sin(q4) + cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) - cos(q5) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) + d4 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) + a2 * cos(q1) * sin(q2) + a3 * cos(q1) * cos(q2) * sin(q3) + a3 * cos(q1) * cos(q3) * sin(q2);
  rt.y = a1 * sin(q1) - d4 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + d6 * (sin(q5) * (cos(q1) * sin(q4) - cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - cos(q5) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) + a2 * sin(q1) * sin(q2) + a3 * cos(q2) * sin(q1) * sin(q3) + a3 * cos(q3) * sin(q1) * sin(q2);
  rt.z = d1 - d4 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - d6 * (cos(q5) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + cos(q4) * sin(q5) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) + a2 * cos(q2) + a3 * cos(q2) * cos(q3) - a3 * sin(q2) * sin(q3);
  rt.r_matrix.n.vx = sin(q6) * (cos(q4) * sin(q1) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) + cos(q6) * (cos(q5) * (sin(q1) * sin(q4) + cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) + sin(q5) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)));
  rt.r_matrix.o.vx = cos(q6) * (cos(q4) * sin(q1) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) - sin(q6) * (cos(q5) * (sin(q1) * sin(q4) + cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) + sin(q5) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)));
  rt.r_matrix.a.vx = cos(q5) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q5) * (sin(q1) * sin(q4) + cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)));
  rt.r_matrix.n.vy = - sin(q6) * (cos(q1) * cos(q4) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - cos(q6) * (cos(q5) * (cos(q1) * sin(q4) - cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) + sin(q5) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)));
  rt.r_matrix.o.vy = sin(q6) * (cos(q5) * (cos(q1) * sin(q4) - cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) + sin(q5) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - cos(q6) * (cos(q1) * cos(q4) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)));
  rt.r_matrix.a.vy = sin(q5) * (cos(q1) * sin(q4) - cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - cos(q5) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
  rt.r_matrix.n.vz = - cos(q6) * (sin(q5) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - cos(q4) * cos(q5) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) - sin(q4) * sin(q6) * (cos(q2) * cos(q3) - sin(q2) * sin(q3));
  rt.r_matrix.o.vz = sin(q6) * (sin(q5) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - cos(q4) * cos(q5) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) - cos(q6) * sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3));
  rt.r_matrix.a.vz = - cos(q5) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - cos(q4) * sin(q5) * (cos(q2) * cos(q3) - sin(q2) * sin(q3));
  return rt;
}
//更新当前关节坐标，笛卡尔坐标，齐次矩阵
void set_current_coord(joints_t a) {
  cur_joint.q1 = a.q1;
  cur_joint.q2 = a.q2;
  cur_joint.q3 = a.q3;
  cur_joint.q4 = a.q4;
  cur_joint.q5 = a.q5;
  cur_joint.q6 = a.q6;
  cur_homo = fkine(cur_joint);
  cur_cart = tr2eul(cur_homo);
  Serial.println("CUR_CART:  ");
  Serial.print(cur_cart.x); Serial.print("  "); Serial.print(cur_cart.y); Serial.print("  "); Serial.print(cur_cart.z); Serial.print("  ");
  Serial.print(cur_cart.a); Serial.print("  "); Serial.print(cur_cart.b); Serial.print("  "); Serial.println(cur_cart.c);
  Serial.print("CUR_JOINT:  "); Serial.print(cur_joint.q1); Serial.print("  "); Serial.print(cur_joint.q2); Serial.print("  "); Serial.print(cur_joint.q3); Serial.print("  ");
  Serial.print(cur_joint.q4); Serial.print("  "); Serial.print(cur_joint.q5); Serial.print("  "); Serial.println(cur_joint.q6);
}

