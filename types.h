struct joints_s{ 
  double q1;
  double q2;
  double q3;
  double q4;
  double q5;
  double q6;
};

struct cart_s{
  double x;
  double y;
  double z;
  double a;
  double b;
  double c;
};
//三维向量
struct vetor_s{
  double vx;
  double vy;
  double vz;
};

//三维矩阵
struct matrix3d_s{
  vetor_s n;
  vetor_s o;
  vetor_s a;
};

//其次矩阵
struct homo_s{
  double x;
  double y;
  double z;
  matrix3d_s r_matrix;
};

struct dyn_s{
  double div_n;
  double vel;
  double acc;
};
/* backup code
  Serial.print("CUR_JOINT:  "); Serial.print(cur_joint.q1); Serial.print("  "); Serial.print(cur_joint.q2); Serial.print("  "); Serial.print(cur_joint.q3); Serial.print("  ");
  Serial.print(cur_joint.q4); Serial.print("  "); Serial.print(cur_joint.q5); Serial.print("  "); Serial.println(cur_joint.q6);
  Serial.println("CUR_HOMO:  ");
  Serial.print(cur_homo.r_matrix.n.vx); Serial.print("  "); Serial.print(cur_homo.r_matrix.o.vx); Serial.print("  "); Serial.print(cur_homo.r_matrix.a.vx); Serial.print("  "); Serial.println(cur_homo.x);
  Serial.print(cur_homo.r_matrix.n.vy); Serial.print("  "); Serial.print(cur_homo.r_matrix.o.vy); Serial.print("  "); Serial.print(cur_homo.r_matrix.a.vy); Serial.print("  "); Serial.println(cur_homo.y);
  Serial.print(cur_homo.r_matrix.n.vz); Serial.print("  "); Serial.print(cur_homo.r_matrix.o.vz); Serial.print("  "); Serial.print(cur_homo.r_matrix.a.vz); Serial.print("  "); Serial.println(cur_homo.z);
  Serial.println("CUR_CART:  ");
  Serial.print(cur_cart.x); Serial.print("  "); Serial.print(cur_cart.y); Serial.print("  "); Serial.print(cur_cart.z); Serial.print("  ");
  Serial.print(cur_cart.a); Serial.print("  "); Serial.print(cur_cart.b); Serial.print("  "); Serial.println(cur_cart.c);
  Serial.println("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");

     Serial.println("JTRAJ:  "); Serial.print(qt.q1); Serial.print("  "); Serial.print(qt.q2); Serial.print("  "); Serial.print(qt.q3); Serial.print("  ");
    Serial.print(qt.q4); Serial.print("  "); Serial.print(qt.q5); Serial.print("  "); Serial.println(qt.q6);



  Serial.println("jg  ");
  Serial.print(n11); Serial.print("  "); Serial.print(n12); Serial.print("  "); Serial.print(n13); Serial.print("  "); Serial.println(n14);
  Serial.print(n21); Serial.print("  "); Serial.print(n22); Serial.print("  "); Serial.print(n23); Serial.print("  "); Serial.println(n24);
  Serial.print(n31); Serial.print("  "); Serial.print(n32); Serial.print("  "); Serial.print(n33); Serial.print("  "); Serial.println(n34);
*/

