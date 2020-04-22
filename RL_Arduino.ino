//Inicjalizacja globalnych zmiennych
short maxTrials = 50;
short maxSteps = 5000;
short i = 0;
short j = 0;
float alpha = 0.7;
float gamma = 0.7;
float T = 5;
float eps = 0.05;
float reward;
float phi = 25;
short nStates = 32;
short nActions = 5;
short state;
short nextState;
short action;
bool ifOutOfRange = false;
bool outOfRange = false;
bool crash = true;
short iterRange = 0;
int* sensors;
int actions[5] = {-2, -1, 0, 1, 2};
int** stateSpace;
float** QTable;

//Zerowanie tablicy Q-wartosci
float** zeroQTable(float** QTable)
{
  int iter = 0;
  for(int i=0; i<32; i++)
    for(int j = 0; j<5; j++)
      QTable[i][j] = 0.0;
  return QTable;
}
//Tworzenie macierzy stanu
int** CreateStateSpace(int** stateSpace)
{
  int x[5] = {0, 0, 0, 0, 0};
  int nK[5] = {2, 2, 2, 2, 2};
  int jiter = 0;
  int iter = 0;
  while(x[0]<nK[0])
  {
    while(x[1]<nK[1])
    {
      while(x[2]<nK[2])
      {
        while(x[3]<nK[3])
        {
          while(x[4] < nK[4])
          {
            for(jiter=0;jiter<5;jiter++)
                stateSpace[iter][jiter] = x[jiter];
            jiter = 0;
            iter++;
            x[4]++;
          }
          x[4] = 0;
          x[3]++;
        }
        x[3]=0;
        x[2]++;
      }
      x[2]=0;
      x[1]++;
    }
    x[1]=0;
    x[0]++;
  }  
  return stateSpace;
}
//Sprawdzanie czujnikow
int* Check(int* sensors)
{
  if(analogRead(A0) >= 600)
    sensors[0] = 1;
  else
    sensors[0] = 0; 
 
  if(analogRead(A1) >= 600)
    sensors[1] = 1;
  else
    sensors[1] = 0; 
 
  if((analogRead(A2) >= 600) || (analogRead(A3) >= 600))
    sensors[2] = 1;
  else
    sensors[2] = 0; 
   
  if(analogRead(A4) >= 600)
    sensors[3] = 1;
  else
    sensors[3] = 0; 
   
  if(analogRead(A5) >= 600)
    sensors[4] = 1;
  else
    sensors[4] = 0; 

  return sensors;
}
//Dyskretyzacja
int Discretization(int* sensors)
{
  int sum = 0;
  for(int it=0; it<32; it++)
  {
    for(int jt=0; jt<5; jt++)
      if(sensors[jt] == stateSpace[it][jt])
        sum+= 1;
    if(sum == 5)
      return it;
    sum = 0;
  }
  return -1;
}
//Wybor akcji
int ChooseAction(float** QTable, int state, float eps)
{
  return EpsilonGreedy(QTable, eps, state);
  //return SoftMaxSelection(QTable);
}
//Wybor akcji ze strategia epsilon-greedy
int EpsilonGreedy(float** QTable, float eps, int state)
{
  float r = (float)random(0, 100)/100.0;
  float maxValue;
  if(r < eps)
    return (int)random(-2, 3);
  
  float sum = 0.0;
  for(int it=0; it<5; it++)
      sum += QTable[state][it];
  if(sum == 0.0)
     return (int)random(-2, 3);
     
  maxValue = GetMax(QTable[state]);
  for(int it=0; it<5; it++)
    if(QTable[state][it] == maxValue)
      return actions[it];
}
//Wybor akcji z rozkladem Boltzmann'a
int SoftMaxSelection(float** QTable)
{
    float P[5];
    float sum = 0;
    
    float r = random(0, 100)/100.0;
    
    //Boltzmann distribution
    for(int it=0; it<5; it++)
      sum += exp(QTable[state][it]/T);
     
    for(int i=0; i<5; i++)
      P[i] = exp(QTable[state][i]/T)/sum;
    
      if(r < P[0])
          return -2;
      else if(r >= P[0] && r < P[1] + P[2])
          return  -1;
      else if(r >= P[0] + P[1] && r < P[0] + P[1] + P[2])
          return  0;
      else if(r >= P[0] + P[1] + P[2] && r < P[0] + P[1] + P[2] + P[3])
          return  1;
      else if(r >= P[0] + P[1] + P[2] + P[3] && r < P[0] + P[1] + P[2] + P[3] + P[4])
          return  2;
}
//Zwracania wartosci maksymalnej z wektora rozmimaru 1x5
float GetMax(float* v)
{
  float maxVal = v[0];
  for(int it=1; it< 5; it++)
    if(v[it] > maxVal)
      maxVal = v[it];
  return maxVal;
}
//Sprawdzanie, czy agent nie wyjechal poza tor
bool CheckIfOutOfRange(int* sensors)
{
  if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0)
    return true;
  return false;
}
//Przyznanie nagrody agentowi
float GetReward(int* sensors, bool outOfRange)
{
  float reward = 0.0;
  if(sensors[0] == 1)
    reward += 0;
  if(sensors[1] == 1)
    reward += 1;
  if(sensors[2] == 1)
    reward += 10;
  if(sensors[3] == 1)
    reward += 1;
  if(sensors[4] == 1)
    reward += 0;
  if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0)
    reward = -20;
  return reward;
}
//Odswiezanie tablicy Q-wartosci
float** UpdateQTable(float** QTable, int state, int nextState, int action, float alpha, float gamma, float reward)
{
  QTable[state][action + 2] = (1 - alpha)*QTable[state][action + 2] + alpha*(reward + gamma*GetMax(QTable[nextState]));
  return QTable;
}
//Wykonywanie akcji
void Move(int action)
{
  if(action == -2)
    TurnLeft2(phi);
  else if(action == -1)
    TurnLeft(phi);
  else if(action == 0)
    GoForward(phi);
  else if(action == 1)
    TurnRight(phi);
  else if(action == 2)
    TurnRight2(phi);
}
//Lagodny zakret w lewo
void TurnLeft(float p)
{
  analogWrite(3, p);
  analogWrite(6, 0);
  delay(50);
}
//Lagody zakret w prawo
void TurnRight(float p)
{
  analogWrite(6, p);
  analogWrite(3, 0);
  delay(50);
}
//Mocny zakret w lewo
void TurnLeft2(float p)
{
  analogWrite(3, 3*p);
  analogWrite(6, 0);
  delay(40);
}
//Mocny zakret w prawo
void TurnRight2(float p)
{
  analogWrite(6, 3*p);
  analogWrite(3, 0);
  delay(40);
}
//Jazda na wprost
void GoForward(float p)
{
  analogWrite(6, 1.5*p);
  analogWrite(3, 1.5*p);
  delay(50);  
}
//Zatrzymanie agenta
void Stop()
{
  analogWrite(3, 0);
  analogWrite(6, 0);
  delay(5000);
}
void setup() 
{
  //Inicjalizacja pinow dla czujnikow wykrywajacych linie
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT); 
  
  //Inicjalizacja pinow dla prawego silnika
  pinMode(3, OUTPUT); 
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  //Wyznaczenie kierunku ruchu obrotowego prawego silnika
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);

  //Inicjalizacja pinow dla lewego silnika
  pinMode(6, OUTPUT); //PWM
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  //Wyznaczenie kierunku ruchu obrotowego lewego silnika
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);

  Serial.begin(9600);

  //Inicjalizacja zmiennych
  sensors = (int*)malloc(5*sizeof(int));
  QTable = (float**)malloc(32*sizeof(float*));
  for(int it = 0; it < 32; it++)
    QTable[it] = (float*)malloc(5*sizeof(float));
  stateSpace = (int**)malloc(32*sizeof(int*));
  for(int it = 0; it < 32; it++)
    stateSpace[it] = (int*)malloc(6*sizeof(int));
  QTable = zeroQTable(QTable);  
  stateSpace = CreateStateSpace(stateSpace);
}

void loop() 
{
  delay(2000);
  i = 0;
  while(i < maxTrials)
  {
    delay(3000);
    //Inicjalizacja zmiennych
    iterRange = 0;
    ifOutOfRange = false;
    j = 0;
    //Poczatkowe ustalenie polozenia agenta wzgledem linii
    sensors = Check(sensors);
    state = Discretization(sensors);

    while(j < maxSteps)
    {
      //Wybor akcji 
      action = ChooseAction(QTable, state, eps);
      //Wykonanie akcji
      Move(action);
      //Sprawdzenie czujnikow
      sensors = Check(sensors);
      //Obliczenie indeksu nowego stanu
      nextState = Discretization(sensors);
      //Sprawdzenie, czy robot nie wyjechal poza tow
      ifOutOfRange = CheckIfOutOfRange(sensors);
      if(ifOutOfRange)
        iterRange++;
      else
        iterRange = 0;
      //Jesli trzykrotnie poza torem, ustaw odpowiednia flage
      if(iterRange == 3)
        outOfRange = true;
      else
        outOfRange = false;
      //Otrzymanie nagrody
      reward = GetReward(sensors, outOfRange);
      //Odswiezenie tablicy Q-wartosci
      QTable = UpdateQTable(QTable, state, nextState, action, alpha, gamma, reward);
      state = nextState;
      //Jesli trzykrotnie poza torem, agent ginie
      if(outOfRange)
      {
        Stop();
        break;
      }
      j++;
    }
    i++; 
  }  
}
