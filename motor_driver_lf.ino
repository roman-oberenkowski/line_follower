#define RIGHT_B 9
#define RIGHT_F 10
#define LEFT_B 5
#define LEFT_F 6
#define LEFT 0
#define RIGHT 1
#define FORWARD 1
#define BACKWARD 0

int tab[2][2];

void set_motor_speed(int lr,int speed);

void setup() {
  pinMode(RIGHT_B,OUTPUT);
  pinMode(RIGHT_F,OUTPUT);
  pinMode(LEFT_B,OUTPUT);
  pinMode(LEFT_F,OUTPUT);
  pinMode(13,OUTPUT);
  tab[LEFT][FORWARD]=LEFT_F;
  tab[LEFT][BACKWARD]=LEFT_B;
  tab[RIGHT][BACKWARD]=RIGHT_B;
  tab[RIGHT][FORWARD]=RIGHT_F;
  Serial.begin(9600);
}


void loop() {
  while (Serial.available() > 0) {
    int left = Serial.parseInt();
    int right = Serial.parseInt();
    if (Serial.read() == '\n') {
      PORTB ^= (1<<PB5);
      int lspeed = scale(left);
      int rspeed = scale(right);
      set_motor_speed(LEFT,lspeed);
      set_motor_speed(RIGHT,rspeed);
      }
   }
   delay(100);
}


int scale(int value){ //-100,100 -> -255,-100 100,255
  if(value==0)return 0;
  value=constrain(value,-100,100);
  if(value<0)value=map(value,-100,0,-254,-100);
  else value=map(value,0,100,100,254);
  value=constrain(value,-255,255);
  Serial.println(value);
  return value;
}

void set_motor_speed(int lr,int speed){
  if(speed>0){
    digitalWrite(tab[lr][BACKWARD],LOW);
    analogWrite(tab[lr][FORWARD],speed);
  } 
  else if(speed<0){
    speed=-speed;
    digitalWrite(tab[lr][FORWARD],LOW);
    analogWrite(tab[lr][BACKWARD],speed);
  } 
  else{
    digitalWrite(tab[lr][FORWARD],LOW);
    digitalWrite(tab[lr][BACKWARD],LOW);
  }
}
