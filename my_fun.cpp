#include "my_fun.h"

unsigned long control_Time;
unsigned long time_start_f_cmd_set = 0;
struct can_frame canMsg, canMsg_read;
float error, integral, derivative;
unsigned long dt = 12;

int dir = 1;
int16_t speed_measure = 0;
int16_t speed_target = 0;
int16_t angle_measure = 0;
int16_t angle_target = 0;
int16_t I_mA = 0;
int k = 0;
float f = 0;
float f_cmd = 0;
String command;
char flag = 's';
char Servo_con_flag = 's';
int myServo_angle = 0;
int offset = 0;
unsigned long Ser_control_Time;
int f_servo100 = 50;

void setup_fun(void) {
  canMsg.can_id = 0x200;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;
  canMsg.data[1] = 0x00;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
  Serial.begin(9600);
  Serial.setTimeout(5);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(20);
  while (flag != 'f') {
    delay(5);
    if (Serial.available() > 0) {
      command = Serial.readStringUntil('\n');
      Serial.println("");
      Serial.println("------ Ready to run-------");
      Serial.println("f100 to set f1.0 hz");
      Serial.println("s to restart");
      Serial.println("command end with newline");
      Serial.print("command:");
      Serial.println(command);
      if (command.startsWith("f")) {
        flag = 'f';
      }
    }
  }
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("running...");
  time_start_f_cmd_set = millis();
  control_Time = millis();
}



int16_t pid_speed_control(int speed_target, int speed_measure) {
  error = speed_target - speed_measure;
  integral += 0.1 * error; /*积分项：误差项的累计*/
  if (integral > 5000) {
    integral = 5000;
  }
  if (integral < -5000) {
    integral = -5000;
  }
  I_mA = int16_t(2.2L * error) + int16_t(integral); /*三项分别乘以PID系数即为输出*/
  return I_mA;
}

int16_t pid_angle_control(int angle_target, int angle_measure) {
  error = angle_target - angle_measure;
  integral += 3 * error; /*积分项：误差项的累计*/
  if (integral > 5000) {
    integral = 5000;
  }
  if (integral < -5000) {
    integral = -5000;
  }
  I_mA = int16_t(50.1L * error) + int16_t(integral); /*三项分别乘以PID系数即为输出*/
  return I_mA;
}

int16_t check_I(int16_t I_mA) {
  if (I_mA > 10000) {
    I_mA = 10000L;
  }
  if (I_mA < -10000) {
    I_mA = -10000L;
  }
  return I_mA;
}

void print_run_msg(void) {
  Serial.println("---------------------");
  Serial.println("f: " + String(f));
  Serial.println("angle_measure= " + String(angle_measure / 8191.0 * 360.0));
  Serial.println("speed_measure= " + String(speed_measure / 36));
  Serial.println("speed_target= " + String(speed_target / 36));
  Serial.println("I_mA:" + String(I_mA));
  Serial.println("dt:" + String(dt));
}

void update_R_measure(void) {
  if (canMsg_read.can_id == 0x201) {
    speed_measure = canMsg_read.data[2];
    speed_measure <<= 8;
    speed_measure |= canMsg_read.data[3];
    angle_measure = canMsg_read.data[0];
    angle_measure <<= 8;
    angle_measure |= canMsg_read.data[1];
  }
}

void update_f(void) {
  if (abs(f - f_cmd) > 0.0001) {
    f = (1 - exp((time_start_f_cmd_set / 50000.0 - millis() / 50000.0))) * (f_cmd - f) + f;
  } else {
    f = f_cmd;
  }
}

void control_R_motor(void) {
  control_Time = millis();
  speed_target = dir * f * 60 * 36; // 36为减速比，250最大转速250rpm
  I_mA = pid_speed_control(speed_target, speed_measure);
  I_mA = check_I(I_mA);
  if (f - 0 < 0.001) {
    I_mA = 0;
  }

  canMsg.data[1] = (I_mA >> (8 * 0)) & 0xff;
  canMsg.data[0] = (I_mA >> (8 * 1)) & 0xff;
  mcp2515.sendMessage(&canMsg);
}
void control_Servo(void) {
  if (Servo_con_flag == 's') {

  } else {
    myServo_angle = 90 + offset + 30 * sin(2 * f_servo100 / 100.0 * PI * (millis() - Ser_control_Time) / 1000.0);
    if (myServo_angle > 180) myServo_angle = 180;
    if (myServo_angle < 0) myServo_angle = 0;
    myServo.write(myServo_angle);
  }

}
void set_parameter(String action) {
  if (action.startsWith("f")) {
    f_servo100 = action.substring(1).toInt(); //
    Serial.println("f_servo100:");
    Serial.println(f_servo100);
  } else {
    Serial.println("Invalid command.");
  }
}
// 根据指令内容执行操作
void processPumpAction(String action) {
  if (action.startsWith("+") || action.startsWith("-")) {
    int pwmValue = action.substring(1).toInt(); // 提取 PWM 占空比
    if (action.startsWith("+")) {
      setPumpForward(pwmValue); // 正转
    } else if (action.startsWith("-")) {
      setPumpReverse(pwmValue); // 反转
    }
  } else {
    Serial.println("Invalid command. Use c+<pwm>, or c-<pwm>.");
  }
}
void processServopAction(String action) {
  if (action.startsWith("+") || action.startsWith("-")) {
    offset = action.substring(1).toInt(); // 提取 PWM 占空比
    Ser_control_Time = millis();
    if (action.startsWith("+")) {
      Servo_con_flag = '+'; // 正转
      offset = offset;
    } else if (action.startsWith("-")) {
      Servo_con_flag = '-';
      offset = -offset;
    }
  } else {
    Servo_con_flag = 's';
    myServo.writeMicroseconds(1500);
    delay(5);
    Serial.println("Invalid command.");
  }
}

// 设置抽水机正转
void setPumpForward(int pwmValue) {
  if (pwmValue < 0 || pwmValue > 100) {
    Serial.println("Invalid PWM value. Must be between 0 and 100.");
    return;
  }
  analogWrite(IN2_PIN, map(pwmValue, 0, 100, 0, 255)); // 设置正转 PWM 占空比
  digitalWrite(IN1_PIN, LOW);     // IN2 设为 LOW
  Serial.print("Pump set to forward with PWM: ");
  Serial.println(pwmValue);
}

// 设置抽水机反转
void setPumpReverse(int pwmValue) {
  if (pwmValue < 0 || pwmValue > 100) {
    Serial.println("Invalid PWM value. Must be between 0 and 100.");
    return;
  }
  digitalWrite(IN1_PIN, HIGH);     // IN1 设为 LOW
  analogWrite(IN2_PIN, map(pwmValue, 0, 100, 0, 255)); // 设置反转 PWM 占空比
  Serial.print("Pump set to reverse with PWM: ");
  Serial.println(pwmValue);
}




void handleSerialCommand(String command) {
  Serial.print("command:");
  Serial.println(command);
  if (command.startsWith("f")) {
    flag = 'f';//设置摆动频率
    f_cmd = (command.substring(1).toInt()) / 100.0;
    time_start_f_cmd_set = millis();
    Serial.print("New f : ");
    Serial.println(f_cmd);
  }
  else if (command.startsWith("s")) {
    flag = 's';//电机停止，并且重启系统
    canMsg.data[1] = (0 >> (8 * 0)) & 0xff;
    canMsg.data[0] = (0 >> (8 * 1)) & 0xff;
    mcp2515.sendMessage(&canMsg);
    delay(100);
    asm volatile ("  jmp 0");
  }
  else if (command.startsWith("t")) {//转弯控制 t1000   t2000
    int pos = command.substring(1).toInt(); // 提取命令中的数字
    if (pos >= 1000 && pos <= 2000) {   // 确保位置在合理范围内
      myServo.writeMicroseconds(pos);     // 设置舵机位置
      Serial.print("Servo moved to: ");
      Serial.println(pos);
    } else {
      Serial.println("Invalid position. Use t1000 to t2000.");
    }
  }
  else if (command.startsWith("u")) {//沉浮控制 u+100  u-0 u+0
    String action = command.substring(1); // 提取指令内容
    processPumpAction(action); // 处理指令
  }
  else if (command.startsWith("p")) {//转向控制 p-10 p+10
    String action = command.substring(1); // 提取指令内容
    processServopAction(action); // 处理指令
  }
  else if (command.startsWith("c")) {//设置参数
    String action = command.substring(1); // 提取指令内容
    set_parameter(action); // 处理指令
  }
  else {
    Serial.println("Invalid command.");
  }
}
