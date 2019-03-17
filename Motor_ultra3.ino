#include <Wire.h>   // библиотека для работы I²C
#include <TroykaIMU.h>    // библиотека для работы с модулями IMU

///////////////////переменные для imu
#define BETA 0.22f    // множитель фильтра
Madgwick filter;    // создаём объект для фильтра Madgwick
Accelerometer accel;    // создаём объект для работы с акселерометром
Gyroscope gyro;   // создаём объект для работы с гироскопом
float gx, gy, gz, ax, ay, az;   // переменные для данных с гироскопов, акселерометров
float yaw, pitch, roll;   // получаемые углы ориентации
float fps = 100;    // переменная для хранения частоты выборок фильтра

///////////////////переменные для моторов
// контакты
#define LSTEP_PIN 2   //шаговый контакт левого мотора
#define LDIR_PIN 3    //контакт направления левого мотора
#define RSTEP_PIN 9   //шаговый контакт правого мотора
#define RDIR_PIN 8    //контакт направления правого мотора
#define STEP_MILLIS 1 //скорость шага

bool LD = LOW;
bool RD = HIGH;

int lsteps = 200000;    //количество шагов левого мотора
int rsteps = 200000;    //количество шагов правого мотора
bool lcheck = false;
bool rcheck = false;

int condition = 2;    //состояние моторов относительно направления движения // 0 - направо, 1 - прямо, 2 - налево, 3 - назад
int vector = 0;  // 0 - вправо, 1 - вверх, 2 - влево, 3 - вниз


//////////////////////гироскоп
bool capture = false;   //флаг для захвата yaw для начала поворота
float yaw1=0;           //зафиксированный угол yaw
float delta_yaw=0;      //разница угла yaw
int x = 0, y = 0, xAim = -2, yAim = -3;

/////////////////////енкодер
int rev = A0;
int last = 0;
int encoder = 0;
int Mylength = 15;


/////////////////////задачи
bool firstCheck = false;
bool secondCheck = false;
int led = 12;

void setup()
{
  Serial.begin(115200);   // открываем последовательный порт
  Serial.println("Begin init...");    // инициализация акселерометра
  accel.begin();    // инициализация акселерометра
  gyro.begin();   // инициализация гироскопа
  Serial.println("Initialization completed");   // выводим сообщение об удачной инициализации
  pinMode(LSTEP_PIN, OUTPUT);
  pinMode(LDIR_PIN, OUTPUT);
  pinMode(RSTEP_PIN, OUTPUT);
  pinMode(RDIR_PIN, OUTPUT);
  digitalWrite(LDIR_PIN, LOW);
  digitalWrite(RDIR_PIN, LOW);
  pinMode(rev,INPUT);
  pinMode(led,OUTPUT);
}

void loop()
{
  unsigned long startMillis = millis();    // запоминаем текущее время
  imu(fps);
  if(millis()>5000)
  {
  if((lsteps == 0) && (rsteps == 0))
  {
    digitalWrite(led,HIGH);
  }
  if(condition == 0)
  {
    if(yaw1-90 <= -180)
    {
      if ((yaw < (yaw1 + 270)) && (yaw > 0))
      {
        if (vector - 1 < 0)
        {
          capture = false;
          vector = vector - 1 + 4;
        }
        else
        {
          capture = false;
          vector = (vector - 1)%4;
        }
      }
    }
    else
    {
      if (yaw < (yaw1 - 90))
      {
        if (vector - 1 < 0)
        {
          capture = false;
          vector = vector - 1 + 4;
        }
        else
        {
          capture = false;
          vector = (vector - 1)%4;
        }
      }
    }
  }
  
  if(condition == 2)
  {
    if (yaw1 + 90 >= 180)
    {
      if ((yaw > (yaw1 - 270)) && (yaw < 0))
      {
        capture = false;
        vector = (vector + 1)%4;
      }
    }
    else
    {
      if (yaw > (yaw1 + 90))
      {
        capture = false;
        vector = (vector + 1)%4;
      }
    }
  }
  
  aim();
  turn();
  MyPosition();
  drive(LD,RD);
  Serial.println("condition\t\tvector\t\tencoder\t\tx\t\ty\t\tLD\t\tRD\t\tyaw1\t\tyaw");
  Serial.print(condition);
  Serial.print("\t\t\t");
  Serial.print(vector);
  Serial.print("\t\t");
  Serial.print(encoder);
  Serial.print("\t\t");
  Serial.print(x);
  Serial.print("\t\t");
  Serial.print(y);
  Serial.print("\t\t");
  Serial.print(LD);
  Serial.print("\t\t");
  Serial.print(RD);
  Serial.print("\t\t");
  Serial.print(yaw1);
  Serial.print("\t\t");
  Serial.println(yaw);
  }
  unsigned long deltaMillis = millis() - startMillis;    // вычисляем затраченное время на обработку данных
  fps = 1000 / deltaMillis;       // вычисляем частоту обработки фильтра
}

void drive(bool ldir, bool rdir)
{
  if (lsteps>0)
  {
    digitalWrite(LDIR_PIN, ldir);
    digitalWrite(LSTEP_PIN, HIGH);
    delay(STEP_MILLIS);
    digitalWrite(LSTEP_PIN, LOW);
    delay(STEP_MILLIS);
    lsteps=lsteps - 1;
  }
  if (rsteps>0)
  {
    digitalWrite(RDIR_PIN, rdir);
    digitalWrite(RSTEP_PIN, HIGH);
    delay(STEP_MILLIS);
    digitalWrite(RSTEP_PIN, LOW);
    delay(STEP_MILLIS);
    rsteps=rsteps - 1;
  }
}

void imu(float fps)
{
  accel.readGXYZ(&ax, &ay, &az);    // считываем данные с акселерометра в единицах G
  gyro.readRadPerSecXYZ(&gx, &gy, &gz);   // считываем данные с акселерометра в радианах в секунду
  filter.setKoeff(fps, BETA);   // устанавливаем коэффициенты фильтра
  filter.update(gx, gy, gz, ax, ay, az);    // обновляем входные данные в фильтр
  yaw = filter.getYawDeg();    // получение углов yaw, pitch и roll из фильтра
}

void turn()       //функция поворота
{
  if(condition == 0)
  {
    LD = LOW;
    RD = LOW;
  }
  if(condition == 1)
  {
    LD = LOW;
    RD = HIGH;
  }
  if(condition == 2)
  {
    LD = HIGH;
    RD = HIGH;
  }
  if(condition == 3)
  {
    LD = HIGH;
    RD = LOW;
  }
}


void MyPosition()
{
  if ((condition != 0) && (condition != 2))
  {
  if ((last <= 700) && (analogRead(rev) > 700))
  {
    last = analogRead(rev);
    encoder++;
  }
  if ((last >= 700) && (analogRead(rev) < 700))
  {
    last = analogRead(rev);
    encoder++;
  }
  }
  if (encoder>=Mylength)
  {
    encoder = 0;
    if((condition != 0) && (condition != 2))
    {
    if (vector == 0)
    {
      x++;
    }
    if (vector == 2)
    {
      x--;
    }
    if (vector == 1)
    {
      y++;
    }
    if (vector == 3)
    {
      y--;
    }
    }
  }
}


void aim()
{
  if (x == xAim)
  {
    
    if (y == yAim)
    {
      if(firstCheck == true)
      {
        if (secondCheck == true)
        {
          stopMotors();
        }
        else
        {
          xAim = 0;
          yAim = 0;
          secondCheck = true;
        }
      }
      else
      {
        xAim = -2;
        yAim = 1;
        firstCheck = true;
      }
    }
    else
    {
      if (y < yAim) // значит цель выше
      {
        //delay(2000);
        if (vector == 2) //смотрит налево
        {
          condition = 0;
          if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
        }
        if (vector == 3) //смотрит вниз
        {
          condition = 0;
          if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
        }
        if (vector == 0) // значит смотрит вправо
        {
          condition = 2;
          if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
        }
        if (vector == 1) // значит смотрит вверх
        {
          condition = 1;
        }
       
      }
      if (y > yAim) // значит цель ниже
      {
        if (vector == 2)
        {
          condition = 2;
          if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
        }
        if (vector == 1)
        {
          condition = 0;
          if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
        }
        if (vector == 0)
        {
          condition = 0;
          if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
        }
        if (vector == 3)
        {
          condition = 1;
        }
      }
    }
  }
  else
  {
    if (x < xAim) // значит цель правее
    {
      if (vector == 1) // значит смотрит вверх
      {
        condition = 0;
        if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
      }
      if (vector == 2) // значит смотрит влево
      {
        condition = 0;
        if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
      }
      if (vector == 3) // значит смотрит вниз
      {
        condition = 2;
        if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
      }
      if (vector == 0) // значит смотрит вправо
      {
        condition = 1;
      }
    }
    if (x > xAim) // значит цель левее
    {
      
      if (vector == 1) // значит смотрит вверх
      {
        condition = 2;
        if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
      }
      if (vector == 2) // значит смотрит налево
      {
        condition = 1;
      }
      if (vector == 3) // значит смотрит вниз
      {
        condition = 0;
        if (capture == false)
          {
            capture = true;
            yaw1=yaw;
          }
      }
      if (vector == 0) // значит смотрит направо
      {
        condition = 0;
        if (capture == false)
        {
          capture = true;
          yaw1=yaw;
        }
      }
    }
  }
}

void stopMotors()
{
  lsteps = 0;
  rsteps = 0;
}

