#include "CarController.h"

Car_typedef AMT_Car;      //AMT_Car

void init_Car(void)
{
   init_Motor();
   AMT_Car.Speed=0;
}

