#include <AFMotor.h> 


AF_DCMotor motor1(1); 
AF_DCMotor motor2(2); 

void setup() 
  {

  }

void loop()
  {
    //Устанавливаем скорость 100% (0-255)
    motor1.setSpeed(255);
    motor2.setSpeed(255);
    
    // Задаем направление движение
    // FORWARD — вперед
    // BACKWARD — назад
    // RELEASE — стоп
    // Движение вперед в течении 5 секунд
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    
    delay(5000);

    // Поворот вправо в течении 2 секунд
    motor1.run(FORWARD);
    motor2.run(BACKWARD);
    
    delay(2000);

    // Движение назад в течении 5 секунд
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);

    delay(5000);

    //Поворот влево в течении 2 секунд
    motor1.run(BACKWARD);
    motor2.run(FORWARD);
    
    delay(2000);

    // Остановка двигателей на 3 секунды
    motor1.run(RELEASE);
    motor2.run(RELEASE);

    delay(3000);

  }
