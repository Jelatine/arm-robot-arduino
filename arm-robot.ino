#include "kinematic.h"
#include <Servo.h>
int sensorPin = A0;
int sensorValue = 0;
bool down = true;
bool E_STOP = false;
int start = 0;
int stp=0;
joints_t cur_joint;
cart_t cur_cart;
homo_t cur_homo;
Servo joint_1, joint_2, joint_3, joint_4, joint_5, joint_6;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(2000000);
  //  cur_cart.x = -100; cur_cart.y = 0;
  robot_home(); //初始
}

void loop() {
  sensorValue = analogRead(sensorPin);
  if (sensorValue > 600) {
    stp++;
    delay(50);
    if(stp>10){
     start=1;
     stp=0;
    }
  }
  if (start == 1) {
    start=0;
    j_a_move(tr_joint(-45, 0, 0, 0, 60, 0));
    delay(100);
    l_r_move(-17, 17, -35);//准备位置
    delay(10);
    l_r_move(30, -30, -18, 0.3);
    j_r_move(0, 0, 0, 0, -45, 0, 0.9);
    delay(10);
    l_r_move(0, 0, 50, 0.3);
    delay(10);
    j_r_move(120, 0, 0, 0, 0, 0, 0.9);
    delay(10);
    l_r_move(15*sin(15*PI/180), 15*cos(15*PI/180), 10, 0.1);
    delay(2000);

    j_a_move(tr_joint(0, -45, 45, 0, 0, 0), 0.5); //回归
    delay(20000);
  }
  if (E_STOP) {

    Serial.println("clear");
    delay(2000);
    clear_error();
  }
}
//关节绝对运动-DEFAULT
void j_a_move(joints_t joint) {
  if (!E_STOP) {
    dyn_t para;
    para.vel = 80;
    jtraj(tr_joint(joint.q1, joint.q2, joint.q3, joint.q4, joint.q5, joint.q6), para);
  }
}
//关节绝对运动-百分百速度-PERCENTAGE
void j_a_move(joints_t joint, double vel_prec) {
  if (!E_STOP) {
    dyn_t para;
    para.vel = 100 * vel_prec;
    jtraj(tr_joint(joint.q1, joint.q2, joint.q3, joint.q4, joint.q5, joint.q6), para);
  }
}
//关节绝对运动-详细参数
void j_a_move(joints_t joint, dyn_t para) {
  if (!E_STOP) {
    jtraj(tr_joint(joint.q1, joint.q2, joint.q3, joint.q4, joint.q5, joint.q6), para);
  }
}

//关节相对运动
void j_r_move(double j1, double j2, double j3, double j4, double j5, double j6) {
  if (!E_STOP) {
    dyn_t para;
    para.vel = 80;
    jtraj(tr_joint(cur_joint.q1 + j1, cur_joint.q2 + j2, cur_joint.q3 + j3, cur_joint.q4 + j4, cur_joint.q5 + j5, cur_joint.q6 + j6), para);
  }
}
//关节相对运动
void j_r_move(double j1, double j2, double j3, double j4, double j5, double j6, double vel_prec) {
  if (!E_STOP) {
    dyn_t para;
    para.vel = 100 * vel_prec;
    jtraj(tr_joint(cur_joint.q1 + j1, cur_joint.q2 + j2, cur_joint.q3 + j3, cur_joint.q4 + j4, cur_joint.q5 + j5, cur_joint.q6 + j6), para);
  }
}
//关节相对运动
void j_r_move(double j1, double j2, double j3, double j4, double j5, double j6, dyn_t para) {
  if (!E_STOP) {
    jtraj(tr_joint(cur_joint.q1 + j1, cur_joint.q2 + j2, cur_joint.q3 + j3, cur_joint.q4 + j4, cur_joint.q5 + j5, cur_joint.q6 + j6), para);
  }
}

/*-------------------Linear直线运动----------------------- */
/*----绝对运动----*/
//直线绝对运动-DEFAULT
void l_a_move(cart_t cart) {
  if (!E_STOP) {
    dyn_t para = {50, 80, 800};
    ctraj(tr_cart(cart.x, cart.y, cart.z , cart.a, cart.b, cart.c), para);
  }
}
//直线绝对运动-百分百速度-PERCENTAGE
void l_a_move(cart_t cart, double vel_prec) {
  if (!E_STOP) {
    if (vel_prec < 0.01)vel_prec = 0.01;
    dyn_t para;
    double ref_x = cart.x - cur_cart.x;
    double ref_y = cart.y - cur_cart.y;
    double ref_z = cart.z - cur_cart.z;
    double s = sqrt(ref_x * ref_x + ref_y * ref_y + ref_z * ref_z);
    para.vel = 500 * vel_prec;
    double T = s / (para.vel * (0.5 + 0.333 * vel_prec));
    para.acc = para.vel / ((0.5 - 0.333 * vel_prec) * T);
    ctraj(tr_cart(cart.x, cart.y, cart.z , cart.a, cart.b, cart.c), para);
  }
}
//直线绝对运动-详细参数
void l_a_move(cart_t cart, dyn_t para) {
  if (!E_STOP) {
    ctraj(tr_cart(cart.x, cart.y, cart.z , cart.a, cart.b, cart.c), para);
  }
}
/*----相对运动----*/
//直线相对运动-默认速度-DEFAULT
void l_r_move(double ref_x, double ref_y, double ref_z) {
  if (!E_STOP) {
    dyn_t para = {50, 80, 800};
    ctraj(tr_cart(cur_cart.x + ref_x, cur_cart.y + ref_y, cur_cart.z + ref_z , cur_cart.a, cur_cart.b, cur_cart.c), para);
  }
}

//直线相对运动-百分百速度-PERCENTAGE
void l_r_move(double ref_x, double ref_y, double ref_z, double vel_prec) {
  if (!E_STOP) {
    if (vel_prec < 0.01)vel_prec = 0.01;
    dyn_t para;
    double s = sqrt(ref_x * ref_x + ref_y * ref_y + ref_z * ref_z);
    para.vel = 500 * vel_prec;
    double T = s / (para.vel * (0.5 + 0.333 * vel_prec));
    para.acc = para.vel / ((0.5 - 0.333 * vel_prec) * T);
    ctraj(tr_cart(cur_cart.x + ref_x, cur_cart.y + ref_y, cur_cart.z + ref_z , cur_cart.a, cur_cart.b, cur_cart.c), para);
  }
}
//直线相对运动-详细参数
void l_r_move(double ref_x, double ref_y, double ref_z, dyn_t para) {
  if (!E_STOP) {
    ctraj(tr_cart(cur_cart.x + ref_x, cur_cart.y + ref_y, cur_cart.z + ref_z , cur_cart.a, cur_cart.b, cur_cart.c), para);
  }
}
/*------------------------------------------------------- */
void c_r_move(double r, int clockwise, double angle ) {
  if (!E_STOP) {
    joints_t qt;
    angle = angle * PI / 180;
    double ndiv = 100;
    double rx, ry, t;
    unsigned long time_s;
    for (double i = 0; i < ndiv + 1; i = i + 1) {
      time_s = micros();
      t = i / ndiv;
      rx = r * cos(angle * t);
      ry = r * sin(angle * t);
      qt = c_ikine(tr_cart(cur_cart.x + rx - r, cur_cart.y + ry, cur_cart.z , cur_cart.a, cur_cart.b, cur_cart.c));
      setJoints(qt);
      while (micros() - time_s < 30000);
    }
    set_current_coord(qt);
  }
}

//圆弧绝对运动
void c_a_move(cart_t end_p, cart_t aux_p) {
  if (!E_STOP) {
    joints_t qt;
    double a1 = cur_cart.x, b1 = cur_cart.y;
    double a2 = end_p.x, b2 = end_p.y;
    double a3 = aux_p.x, b3 = aux_p.y;
    double k1, k2, D, E, F, R, x0, y0;
    k1 = (a2 * a2 + b2 * b2 - a1 * a1 - b1 * b1) * (a3 - a1) - (a3 * a3 + b3 * b3 - a1 * a1 - b1 * b1) * (a2 - a1);
    k2 = (b3 - b1) * (a2 - a1) - (b2 - b1) * (a3 - a1);
    E = k1 / k2;
    if (fabs(a2 - a1) < 0.001) {
      k1 = -(b3 - b1) * E - (a3 * a3 + b3 * b3 - a1 * a1 - b1 * b1);
      k2 = a3 - a1;
    }
    else {
      k1 = -(b2 - b1) * E - (a2 * a2 + b2 * b2 - a1 * a1 - b1 * b1);
      k2 = a2 - a1;
    }
    D = k1 / k2;
    F = E * b1 - a1 * a1 - b1 * b1 - D * a1;
    R = 0.5 * sqrt(D * D + E * E - 4 * F);
    x0 = -0.5 * D; y0 = -0.5 * E;
    double n1, n2, n3, i, th,  dx, dy, nx = 50.0;
    n1 = atan2(b1 - y0, a1 - x0);
    n2 = atan2(b2 - y0, a2 - x0);
    n3 = atan2(b3 - y0, a3 - x0);
    unsigned long time_s;
    for (i = 0; i < nx; i = i + 1.0) {
      time_s = micros();
      if ((n3 > n2 && n3 < n1) || (n3 < n2 && n3 > n1)) {
        th = n1 + (n2 - n1) * i / (nx - 1);
      }
      else if (n1 < n2) {
        th = n1 + (n2 - n1 - 2 * PI) * i / (nx - 1);
      }
      else {
        th = n1 + (n2 - n1 + 2 * PI) * i / (nx - 1);
      }
      dx = R * cos(th) + x0;
      dy = R * sin(th) + y0;
      qt = c_ikine(tr_cart(dx, dy, cur_cart.z , cur_cart.a, cur_cart.b, cur_cart.c));
      setJoints(qt);
      //           Serial.println("JTRAJ:  "); Serial.print(qt.q1); Serial.print("  "); Serial.print(qt.q2); Serial.print("  "); Serial.print(qt.q3); Serial.print("  ");
      //    Serial.print(qt.q4); Serial.print("  "); Serial.print(qt.q5); Serial.print("  "); Serial.println(qt.q6);
      while (micros() - time_s < 30000);
    }
    set_current_coord(qt);
  }
}

//关节参数转换（6AXIS-joints_t）
joints_t tr_joint(double j1, double j2, double j3, double j4, double j5, double j6) {
  joints_t rt = {j1, j2, j3, j4, j5, j6};
  return rt;
}
//笛卡尔参数转换
cart_t tr_cart(double x, double y, double z, double a, double b, double c) {
  cart_t rt = {x, y, z, a, b, c};
  return rt;
}

void robot_home() {
  joint_1.attach(3);
  joint_1.write(0 + 80);
  delay(200);
  joint_2.attach(5);
  joint_2.write((-45) + 90);
  delay(200);
  joint_3.attach(6);
  joint_3.write(-(45) + 90);
  delay(200);
  joint_4.attach(9);
  joint_4.write(0 + 90);
  delay(200);
  joint_5.attach(10);
  joint_5.write(0 + 90);
  set_current_coord(tr_joint(0, -45, 45, 0, 0, 0));
}

void clear_error() {
  E_STOP = false;
  j_a_move(tr_joint(0, -45, 45, 0, 0, 0));
}

//转动六个关节
int setJoints(joints_t arr) {
  double j1,  j2,  j3,  j4,  j5,  j6;
  j1 = (int)(arr.q1 + 80); j2 = (int)(arr.q2 + 90); j3 = (int)(-arr.q3 + 90); j4 = (int)(arr.q4 + 90); j5 = (int)(-arr.q5 + 90); j6 = (int)(arr.q6 + 90);
  if (!E_STOP && (arr.q1 > 80 || arr.q1 < -80 || arr.q2 > 80 || arr.q2 < -50 || arr.q3 > 50 || arr.q3 < -80 || arr.q4 > 85 || arr.q4 < -85 || arr.q5 > 90 || arr.q5 < -90)) {
    Serial.println("ERROR!");
    if (arr.q1 > 80 || arr.q1 < -80)Serial.println("J1 ERROR!");
    if (arr.q2 > 80 || arr.q2 < -50)Serial.println("J2 ERROR!");
    if ( arr.q3 > 50 || arr.q3 < -80)Serial.println("J3 ERROR!");
    if (arr.q4 > 89 || arr.q4 < -89)Serial.println("J4 ERROR!");
    if (arr.q5 > 90 || arr.q5 < -90)Serial.println("J5 ERROR!");
    Serial.print("s:"); Serial.print(arr.q1); Serial.print(","); Serial.print(arr.q2); Serial.print(","); Serial.print(arr.q3); Serial.print(",");
    Serial.print(arr.q4); Serial.print(","); Serial.print(arr.q5); Serial.print(","); Serial.println(arr.q6);
    E_STOP = true;
    return -1;
  }
  if (!E_STOP) {
    joint_1.write(j1);
    joint_2.write(j2);
    joint_3.write(j3);
    joint_4.write(j4);
    joint_5.write(j5);
    joint_6.write(j6);
    cur_joint.q1 = arr.q1; cur_joint.q2 = arr.q2; cur_joint.q3 = arr.q3; cur_joint.q4 = arr.q4; cur_joint.q5 = arr.q5; cur_joint.q6 = arr.q6;
    //    Serial.print("s:"); Serial.print(cur_joint.q1); Serial.print(","); Serial.print(cur_joint.q2); Serial.print(","); Serial.print(cur_joint.q3); Serial.print(",");
    //    Serial.print(cur_joint.q4); Serial.print(","); Serial.print(cur_joint.q5); Serial.print(","); Serial.println(cur_joint.q6);
  }
  return 1;
}

void jtraj(joints_t arr1, dyn_t dyn_para) {
  Serial.println("jtraj:");
  //double tv = dyn_para.div_n;
  //unsigned long dtime = (unsigned long)(20000.0 - dyn_para.vel * 10000.0);
  joints_t arr0 = cur_joint;
  double dtq1 = arr1.q1 - arr0.q1;
  double dtq2 = arr1.q2 - arr0.q2;
  double dtq3 = arr1.q3 - arr0.q3;
  double dtq4 = arr1.q4 - arr0.q4;
  double dtq5 = arr1.q5 - arr0.q5;
  double dtq6 = arr1.q6 - arr0.q6;
  double maxAngle = max(max(max(fabs(dtq1), fabs(dtq2)), fabs(dtq3)), max(max(fabs(dtq4), fabs(dtq5)), fabs(dtq6)));
  Serial.print("MAX ANGLE= "); Serial.println(maxAngle);
  double tv = maxAngle / dyn_para.vel;
  Serial.print("tv= "); Serial.println(tv);
  joints_t qt;
  setJoints(arr0);
  double t;
  unsigned long time_s;
  for (double i = 0; i < tv - 0.01; i = i + 0.01) {
    time_s = micros();
    t = i / (tv - 0.01);
    qt.q1 = pow(t, 5) * 6 * (arr1.q1 - arr0.q1) + pow(t, 4) * (-15) * (arr1.q1 - arr0.q1) + pow(t, 3) * 10 * (arr1.q1 - arr0.q1) + arr0.q1;
    qt.q2 = pow(t, 5) * 6 * (arr1.q2 - arr0.q2) + pow(t, 4) * (-15) * (arr1.q2 - arr0.q2) + pow(t, 3) * 10 * (arr1.q2 - arr0.q2) + arr0.q2;
    qt.q3 = pow(t, 5) * 6 * (arr1.q3 - arr0.q3) + pow(t, 4) * (-15) * (arr1.q3 - arr0.q3) + pow(t, 3) * 10 * (arr1.q3 - arr0.q3) + arr0.q3;
    qt.q4 = pow(t, 5) * 6 * (arr1.q4 - arr0.q4) + pow(t, 4) * (-15) * (arr1.q4 - arr0.q4) + pow(t, 3) * 10 * (arr1.q4 - arr0.q4) + arr0.q4;
    qt.q5 = pow(t, 5) * 6 * (arr1.q5 - arr0.q5) + pow(t, 4) * (-15) * (arr1.q5 - arr0.q5) + pow(t, 3) * 10 * (arr1.q5 - arr0.q5) + arr0.q5;
    qt.q6 = pow(t, 5) * 6 * (arr1.q6 - arr0.q6) + pow(t, 4) * (-15) * (arr1.q6 - arr0.q6) + pow(t, 3) * 10 * (arr1.q6 - arr0.q6) + arr0.q6;
    if (setJoints(qt) == -1) {
      return;
    }
    while (micros() - time_s < 10000);
    //Serial.println(micros() - time_s);
  }
  set_current_coord(qt);
}

void ctraj(cart_t c1, dyn_t dyn_para) {
  Serial.println("ctraj:");
  cart_t c0 = cur_cart;   //起始坐标为当前坐标
  joints_t joints;
  double q0[3] = {c0.x, c0.y, c0.z}; double q1[3] = {c1.x, c1.y, c1.z};
  double vi = dyn_para.vel;//60; //mm/s
  double ai = dyn_para.acc;//400; //mm/s^2
  double path = sqrt((q1[0] - q0[0]) * (q1[0] - q0[0]) + (q1[1] - q0[1]) * (q1[1] - q0[1]) + (q1[2] - q0[2]) * (q1[2] - q0[2]));  //总路程
  double tv = (path / vi + vi / ai); //总耗时
  //  Serial.print("Distance = "); Serial.println(path);
  //  Serial.print("Tims = "); Serial.println(tv);
  double tb = (vi / ai) ;   //加速时间
  if (tb > 0.5 * tv) { //加速太慢
    tb = 0.5 * tv;
  }
  double tf = tv - 0.01;  //减去10ms
  //  Serial.print("YB = "); Serial.println(tb);
  double v[3] = {vi*(q1[0] - q0[0]) / path , vi*(q1[1] - q0[1]) / path , vi*(q1[2] - q0[2]) / path };
  double a[3] = {ai*(q1[0] - q0[0]) / path ,  ai*(q1[1] - q0[1]) / path ,  ai*(q1[2] - q0[2]) / path };
  double s[3];
  unsigned long time_s;
  for (double i = 0; i < tv ; i = i + 0.01) {
    time_s = micros();
    if (i <= tb) {  //加速段
      s[0] = q0[0] + 0.5 * a[0] * i * i;  //X
      s[1] = q0[1] + 0.5 * a[1] * i * i;  //Y
      s[2] = q0[2] + 0.5 * a[2] * i * i;  //Z
    }
    else if (i <= (tv - tb)) {  //匀速段
      s[0] = (q1[0] + q0[0] - v[0] * tf) / 2 + v[0] * i;
      s[1] = (q1[1] + q0[1] - v[1] * tf) / 2 + v[1] * i;
      s[2] = (q1[2] + q0[2] - v[2] * tf) / 2 + v[2] * i;
    }
    else {    //减速段
      s[0] = q1[0] - a[0] / 2 * tf * tf + a[0] * tf * i - a[0] / 2 * i * i;
      s[1] = q1[1] - a[1] / 2 * tf * tf + a[1] * tf * i - a[1] / 2 * i * i;
      s[2] = q1[2] - a[2] / 2 * tf * tf + a[2] * tf * i - a[2] / 2 * i * i;
    }
    c0.x = s[0]; c0.y = s[1]; c0.z = s[2];
    joints = c_ikine(c0);
    if (setJoints(joints) == -1)
      return;
    while (micros() - time_s < 10000);
    //Serial.println(micros() - time_s);
  }
  set_current_coord(joints);
}





