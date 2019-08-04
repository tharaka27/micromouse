// Pin Definitions
// Motor pins - defined whilst looking from rear of robot.
#define mL 10 // Left motor drive
#define mR 11 // Right motor drive
#define enableML 9// Left motor enable pin (High is on)
#define enableMR 12 // Right motor enable pin (High is on)
// Encoder pins
#define encoderLB 2 // Left motor encoder blue pin
#define encoderLG 6 // Left motor encoder green pin
#define encoderRB 3 // Right motor encoder blue pin
#define encoderRG 7 // Right motor encoder green pin
// IR pins
#define FIR A1 // Front IR sensor on ADC 0
#define LIR A2 // Left IR sensor on ADC 1
#define RIR A0 // Right IR sensor on ADC 2
#define IROn 4 // IR emitter pin
// LED pins
#define red 7     // Red LED pin
#define yellow 6  // Yellow LED pin
#define green 5   // Green LED pin

// Constants
#define squareWidth 515
#define degrees90 178


// Cardinal Direction constants
#define north 0
#define east  1
#define south 2
#define west  3

// Global variables
// Setup
byte state = 0;

// Movement
volatile int encoderLBCount = 0; // Contains encoder value for left motor blue wired encoder
volatile int encoderLGCount = 0; // Contains encoder value for left motor green wired encoder
volatile int encoderRBCount = 0; // Contains encoder value for right motor blue wired encoder
volatile int encoderRGCount = 0; // Contains encoder value for right motor green wired encoder
volatile boolean comparisonFlag = 1; // flag used to check comparisons not interrupted

// DFA
byte stackPointer = 0;
byte stack[36][2] = {0}; // Stack for previously visited nodes
boolean maze[8][8] = {0}; // visited matrix
byte edgeMatrix[8][8][4] = {0}; // Matrix to hold wall values

byte x = 0; // x maze coordinate
byte y = 0; // y maze coordinate
byte orientation = 0; // Current orientation of the robot, 0 = north, 1 = east, 2 = south, 3 = west

// A*
byte shortestPath[36][2] = {0}; // Shortest path array, comtains X and Y coords of the path

void setup()
{
  // Setup
  pinMode(enableMR, OUTPUT);
  pinMode(enableML, OUTPUT);
  digitalWrite(enableML, LOW); // These go low so early to try to stop robot running away on startup
  digitalWrite(enableMR, LOW);
  pinMode(mL, OUTPUT);
  pinMode(mR, OUTPUT);
  pinMode(13, OUTPUT); // Pin powering IR sensor recievers
  pinMode(IROn, OUTPUT); // Pin powering IR sensor Emitters
  digitalWrite(13, HIGH);
  digitalWrite(IROn, LOW);

  pinMode(encoderLB, INPUT);
  pinMode(encoderLG, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderRG, INPUT);

  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);

  interrupts();
// Attach interrupts to all the encoder inputs
  attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLBCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderLG), encoderLGCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRBCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRG), encoderRGCounter, RISING);

  // Code
  Serial.begin(9600);
}

void loop()
{
  switch (state) // Main state machine, uses the IR sensors to select a mode
  {
    case 0: // Do nothing state, stops it accidentaly beginning a search
      {
        digitalWrite(red, HIGH);
        if (leftSensor() > 130) // Waits for you to brush an object infront of a sensor
        {
          state = 1; // Set state to intermediate state
        }
        break;
      }
    case 1: // Intermediate state, to stop the modes cycling at clock speed
      {
        if (leftSensor() < 100) // Waits for object to be moved away
        {
          digitalWrite(red, LOW);
          state = 2; // Enter search mode
        }
        break;
      }
    case 2: // Search mode state
      {
        digitalWrite(yellow, HIGH);
        if (rightSensor() > 130) // If object is put infront of right sensor, preform a search
        {
          delay(1000); // Wait a second for object to be moved away
          DFA(); // Search
          turn(north); // After search has been preformed, turn so robot is facing square exit
          digitalWrite(yellow, LOW);
          state = 4; // Enter A* state
        }
        else if (leftSensor() > 130) // Else wait for an object to be placed infront of the left sensor to change mode
        {
          state = 3;
        }
        break;
      }
    case 3: // Intermediate mode to stop mode cycling at clock speed
      {
        if (leftSensor() < 100)
        {
          digitalWrite(yellow, LOW);
          state = 4;
        }
        break;
      }
    case 4: // A* (PAthfinding state) state
      {
        digitalWrite(green, HIGH);
        if (rightSensor() > 130) //Perform A* if object placed infront of right sensor
        {
          delay(1000);
          AStar();
        }
        else if (leftSensor() > 130) // Go back to do nothing state if object placed infront of left sensors
        {
          state = 5;
        }
        break;
      }
    case 5: // Intermediate state to stop state cycling at clock speed
      {
        if (leftSensor() < 100)
        {
          digitalWrite(green, LOW);
          state = 0;
        }
        break;
      }
  }

}

//DFA
void DFA()
{
  int findX = 0;
  int findY = 0;
  boolean flag = 0;

  while (!flag)
  {
    Serial.println();
    /*
    If the square has never been visited before the sensors must be read to check for walls.
    This only needs to happen the first time as readings are assumed correct
    */
    if (maze[x][y] == 0) // Check to see if node is unvisited
    {
      sensorRead(); // Sense walls
      maze[x][y] = 1; // set node to visited
    }
    Serial.println(); // Print over serial for debugging
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.println(y);
    Serial.print("visited? ");
    Serial.println(maze[x][y]);
    Serial.print("north wall? ");
    Serial.println(edgeMatrix[x][y][north]);
    Serial.print("east wall? ");
    Serial.println(edgeMatrix[x][y][east]);
    Serial.print("south wall? ");
    Serial.println(edgeMatrix[x][y][south]);
    Serial.print("west wall? ");
    Serial.println(edgeMatrix[x][y][west]);
    /*
    Once maze data has been collected for the current square, the robot must decide what square to move to next.
    It will always try to go north first, if it is unable to (Either there is a wall in the way or the node has already been visited)
    It will then try to go east, then south then west.
    As it travels it saves the squares it has been to in a stack, so if it hits a dead end it can back track it's steps 
    until it can travel into a new square.IT repeats this process untill it returns to the start square (0,0)
    */
    if ((edgeMatrix[x][y][north] == 1) && (maze[x + 1][y] == 0)) // IF there is a path north and the north square is unvisited
    {
      turn(north); // move there
      orientation = north;
      forward(1);
      x = x + 1; // update coordinates
      stackPointer++; // Save the coordinates to the stack
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][east] == 1) && (maze[x][y + 1] == 0)) // Else try to move east
    {
      turn(east);
      orientation = east;
      forward(1);
      y = y + 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][south] == 1) && (maze[x - 1][y] == 0)) // Else try to move south
    {
      turn(south);
      orientation = south;
      forward(1);
      x = x - 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][west] == 1) && (maze[x][y - 1] == 0)) // Else try to move west
    {
      turn(west);
      orientation = west;
      forward(1);
      y = y - 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else // Other wise pop the stack and travel back a square
    {
      stackPointer--;
      // Only one coordinate can change every square moved, therefore to find the direction
      // to travel the coordinated to move to are subtracted from the current coordinates.
      // This then gives the direction to travel as follows.
      // X1 - X2 = -1 --> move north
      // x1 - x2 = 1  --> move south
      // y1 - y2 = 1  --> move west
      // y1 - y2 = -1 --> move east
      findX = x - stack[stackPointer][0]; // calculate X change
      findY = y - stack[stackPointer][1]; // Calculate Y change
      
      Serial.print("x(-1): "); // Print debugging infomation
      Serial.print(stack[stackPointer][0]);
      Serial.print(" y(-1): ");
      Serial.println(stack[stackPointer][1]);

      if (findX == -1) // move north
      {
        turn(north);
        orientation = north;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findX == 1) // move south
      {
        turn(south);
        orientation = south;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findY == -1) // move east
      {
        turn(east);
        orientation = east;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findY == 1) // move west
      {
        turn(west);
        orientation = west;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
    }
    if((x == 0) && (y == 0)) // Check to see if the robot has returned to the beginning. If it has exit search
    {
      flag = 1;
    }
  }
}

void addEdge(int bearingToChange) 
{
  /*
  Adds a path between two squares. Takes an orientation as input and removes the wall in that direction
  */
  Serial.print("bearingToChange: "); // print debugging info
  Serial.println(bearingToChange);
  
  edgeMatrix[x][y][bearingToChange] = 1; // Removes the wall in the orientation given for the current square
  // As each square has a bit for each wall, the wall must also be removed in the adjacent square.
  if (bearingToChange == north) // If removing north wall, remove southern wall in the northern square
  {
    edgeMatrix[x + 1][y][south] = 1;
  }
  else if (bearingToChange == east) // If removing east wall, remove west wall in the eastern square
  {
    edgeMatrix[x][y + 1][west] = 1;
  }
  else if (bearingToChange == south) // If removing south wall, remove north wall in southern square
  {
    edgeMatrix[x - 1][y][north] = 1;
  }
  else if (bearingToChange == west) // If removing west wall, remove east wall in western square
  {
    edgeMatrix[x][y - 1][east] = 1;
  }
}

void sensorRead()
{
  /*
  Tells sensors to take a reading, it then checks to see fi the reading is below the threshold ADC value found in testing.
  The threshold value is the value a sensor reads if a wall is 6cm away (The maximum distance the robot can be from a wall.
  If the reading is less than the threshold it removes the wall by calling addEdge() and gives it the direction to change.
  */
  int bearingToChange;

  if (frontSensor() < 16)
  {
    Serial.print("adding front edge: ");
    Serial.println(orientation);
    addEdge(orientation); // The front sensor is always facing the direction the robot is facing
  }
  /*
  To calculate the direction the left and right sensors are facing 1 is added or subtracted from the forward direction. 
  If this is outside of the range of the defined directions at the top it wraps the value round before calling addEdge()
  */
  if (leftSensor() < 16)
  {
    Serial.println("Adding left edge");
    bearingToChange = orientation - 1;
    if (bearingToChange == -1)
    {
      bearingToChange = 3;
    }
    addEdge(bearingToChange);
  }
  if (rightSensor() < 16)
  {
    Serial.println("Adding right edge");
    bearingToChange = orientation + 1;
    if (bearingToChange == 4)
    {
      bearingToChange = 0;
    }
    Serial.print("bearing to change: ");
    Serial.println(bearingToChange);
    addEdge(bearingToChange);
  }
}

int frontSensor() 
{
  /*
  Takes a raw reading of the reflected light.
  It does this by turning the IR LED off and taking a background reading, It then turns the LED back on and takes 
  another reading.
  It then returns the background reading subtracted from the reflected reading.
  */
  digitalWrite(IROn, LOW); // Turn LED off (it should already be off) 
  delay(50);
  int x = analogRead(FIR); // Take a background reading
  delay(50);
  digitalWrite(IROn, HIGH); // Turn LED back on
  delay(50);
  int y = analogRead(FIR); // Take reflected light reading
  digitalWrite(IROn, LOW); // Turn LED back off
  Serial.print("FIR");
  Serial.println(y - x);
  return (y - x -2);

}

int leftSensor()
{
  digitalWrite(IROn, LOW);
  delay(50);
  int x = analogRead(LIR);
  delay(50);
  digitalWrite(IROn, HIGH);
  delay(50);
  int y = analogRead(LIR);
  digitalWrite(IROn, LOW);
  Serial.print("LIR");
  Serial.println(y - x);
  return (y - x -2);
}

int rightSensor()
{
  digitalWrite(IROn, LOW);
  delay(50);
  int x = analogRead(RIR);
  delay(50);
  digitalWrite(IROn, HIGH);
  delay(50);
  int y = analogRead(RIR);
  digitalWrite(IROn, LOW);
  Serial.print("RIR");
  Serial.println(y - x);
  return (y - x - 2);
}

//Movement
void turn( byte bearing) // bearing = 0(north),1(east),2(south),3(west)
{
  /*
  Calculates the number of turns the robot has to make to be facing a specific direction.
  It does this by subtracting the direction you wish to face from the current robot direction.
  If the robot calculates 3 turns in a direction are needed the function changes this to one turn in the opposite direction as this is equivilent.
  */
  int turns = orientation - bearing; // Calculate # of turns
  
  Serial.print("turns prefix: "); // Debugging code
  Serial.println(turns);
  
  if (turns == 0) // If the robot calculates 0 turns then the robot can just return to where this function was called
  {
    return;
  }
  else if (turns == 3) // 3 anti-clockwise turns are changed to one clockwise turn
  {
    turns = -1;
  }
  else if (turns == -3) // 3 clockwise turns are converted to one anti-clockwise turn
  {
    turns = 1;
  }

  Serial.print("turns postfix: "); // Print fixed # of turns for debugging
  Serial.println(turns);
  
  if (turns > 0)
  {
    antiClockwise90(turns);
  }
  if (turns < 0)
  {
    clockwise90(abs(turns));
  }
  return;
}

void forward( int squares)
{
  // Travels forward x squares, x being int squares.
  byte i = 0;
  int difference = 0; // difference between left and right motor counts
  byte speedL = 195; // Base speed of left motor
  byte speedR = 191; // Base speed of right motor
  boolean flag1 = 1; // flags used to tell mototrs to stop.
  boolean flag2 = 1;

  for (i = 0; i < squares; i++) // The program to drive forwards 1 square is looped as many times as you need to move forward squares.
  {
    analogWrite(mL, speedL); // Set the speed of the motors to be the default speed set above
    analogWrite(mR, speedR);
    encoderLBCount = 0; // Reset all the encoder counts, the interrupts do not turn off between movements as it turns the timer interrupts off too. which the delays are based off
    encoderLGCount = 0;
    encoderRBCount = 0;
    encoderRGCount = 0;
    flag1 = 1; // Reset the stop flags to high between movements, or robot will only move forward once
    flag2 = 1;
    digitalWrite(enableML, HIGH); // Turn the motor enables on
    digitalWrite(enableMR, HIGH);
    while (flag1 | flag2) // Keeps the loop going until both stop conditions are met.
    {
      speedL = 195; // the speed that each motor needs to travel at is reset at the beginning of the loop
      speedR = 191; // This default value travels through the loop to get the actual speed needed before being written to the motor at the end
      // Stop condidtion
      comparisonFlag = 1; // The comparison flag is used on all if's that use the int variable type. This is due to these needed two cycles to calculate and allowing them to be interrupted by the encoders ruining their result.
      if ((encoderLBCount >= (squareWidth + 4)) && (flag1)) // Stop condition for the left motor
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableML, LOW); // Turn the motor enable off (Tells the motor driver chip to stop the motor)
          flag1 = 0; // set the stop condition for this motor
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (squareWidth)) && (flag2)) // stop condidion for the right motor
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW); // Turn the motor enable off (Tells the motor driver chip to stop the motor)
          flag2 = 0; // set the stop condition for this motor
        }
      }
      // Slow down when coming to stop condition as to not overshoot
      comparisonFlag = 1;
      if ((encoderLBCount >= (squareWidth - 25)) && (flag1))
      {
        if (comparisonFlag == 1)
        {
          speedL = speedL - 30; // If the left motor is near the count goal slow it's speed by 30
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (squareWidth - 25)) && (flag2))
      {
        if (comparisonFlag == 1)
        {
          speedR = speedR - 30; // if the right motor count is near it's goal, slow the right motor 30
        }
      }
      /*
      Motor Control:
      To keep the motors travelling at the same speed the difference between the two encoder counts is multiplied by 
      a static gain and applied to the left motors speed. This means that the further the two counts get from each other the faster or slower 
      the left motor spins to attempt to keep them aligned. the gain of 5 is quite aggressive, as can be seen in person with the robot
      wobbling a bit in the straights
      */
      difference = encoderRBCount - encoderLBCount;
      comparisonFlag = 1;
      if((difference > 0) &&(flag2))
      {
        if(comparisonFlag)
        {
          speedR = speedR - (5 * difference);
          if(speedR < 0) // do not let the speed go below the minimum value that can be written using analogWrite
          {
            speedR = 0;
          }
        }
      }
      comparisonFlag = 1;
      if((difference < 0) && (flag2))
      {
        if(comparisonFlag)
        {
          speedR = speedR - (5 * difference);
          if(speedR > 255) // do not let the speed go above the maximum value that can be written using analogWrite
          {
            speedR = 255;
          }
        }
      }
      analogWrite(mL, speedL); // Write the new calculated speed to both of the motors
      analogWrite(mR, speedR);
    }
  }
  return;
}

void clockwise90( int turns)
{
  /*
  The clockwise90 function commands the motors via the motor driver to make a 90 degree clockwise turn.
  It works in a very similar way to the forward function except that the motor control system was not implemented.
  */
  byte speedL = 169;
  byte speedR = 95;
  boolean flag1 = 1;
  boolean flag2 = 1;
  byte i = 0;

  for (i = 0; i < turns; i++) // Rerun the 90 degree turn for input # of times
  {
    analogWrite(mL, 165); // Write the default speed to the motors for speed calculations
    analogWrite(mR, 95);
    encoderLBCount = 0; // Reset encoder counts as they are still active during all other code
    encoderLGCount = 0;
    encoderRBCount = 0;
    encoderRGCount = 0;
    flag1 = 1; // Reset stop flags
    flag2 = 1;
    digitalWrite(enableML, HIGH); // Enable both motors
    digitalWrite(enableMR, HIGH);
    
    while (flag1 | flag2) // stop the loop when both motors have travelled their set distance
    {
      speedL = 165; // Write the default speed to the motors for speed calculations
      speedR = 95;
      comparisonFlag = 1;

      // Stop if goal met.
      if ((encoderLBCount >= (degrees90)) && (flag1)) 
      {
        if (comparisonFlag == 1) // The comparason flag is used again for these If's as the encoder counts are integers that take 2 cycles to evaluate and can be interrupted ruining the result
        {
          digitalWrite(enableML, LOW); // Write the enable for the left motor low once it's reached it's target
          flag1 = 0; // set the left motor travel as complete
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90)) && ( flag2)) 
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW); // Write the right motor low if it has met it's target
          flag2 = 0; // Set right motor travel as complete
        }
      }
      // If coming close to goal, slow down to not overshoot
      comparisonFlag = 1;
      if ((encoderLBCount >= (degrees90 - 25)) && (flag1)) // as little overshoot is needed as possible, therefore the motors should be set to drive very slowly  just before the target distance is met.
      {
        if (comparisonFlag)
        {
          speedL = speedL - 5; // Turns are made much slower than forward movements, hence only 5 is taken away not 30
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90 - 25)) && (flag2)) // Do the same for the right motor as was done to the left motor above
      {
        if (comparisonFlag)
        {
          speedR = speedR + 5; // Adding 5 to the speed as the right motor is in reverse this time
        }
      }
      analogWrite(mL, speedL); // Update motor speeds
      analogWrite(mR, speedR);
    }
  }
  return;
}

void antiClockwise90( int turns)
{
  /*
  The aniclockwise function is identical to the clockwise90 function, but with the speeds swapped.
  Please read the clockwise90 comments.
  */
  byte speedL = 95;
  byte speedR = 163;
  boolean flag1 = 1;
  boolean flag2 = 1;
  byte i = 0;

  for (i = 0; i < turns; i++)
  {
    analogWrite(mL, speedL);
    analogWrite(mR, speedR);
    encoderLBCount = 0;
    encoderLGCount = 0;
    encoderRBCount = 0;
    encoderRGCount = 0;
    flag1 = 1;
    flag2 = 1;
    digitalWrite(enableML, HIGH);
    digitalWrite(enableMR, HIGH);
    while (flag1 | flag2)
    {
      speedL = 95;
      speedR = 163;
      comparisonFlag = 1;

      // Stop if goal met.
      if ((encoderLBCount >= (degrees90)) && (flag1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90)) && ( flag2))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      // If coming close to goal, slow down to not overshoot
      comparisonFlag = 1;
      if ((encoderLBCount >= (degrees90 - 25)) && (flag1))
      {
        if (comparisonFlag)
        {
          speedL = speedL + 5;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90 - 25)) && (flag2))
      {
        if (comparisonFlag)
        {
          speedR = speedR - 5;
        }
      }
      // Keep count difference to a minimum.
      analogWrite(mL, speedL);
      analogWrite(mR, speedR);
    }
  }
  return;
}
/*
Encoder ISR's 
Simply adds one to each count when called.
Comprison flag is set low in these functions so that it can be determinted if an if statement was interrupted midway through
These can be seen in the movement functions above
*/
void encoderLBCounter( void )
{
  encoderLBCount++;
  comparisonFlag = 0;
}

void encoderLGCounter( void )
{
  encoderLGCount++;
  comparisonFlag = 0;
}

void encoderRBCounter( void )
{
  encoderRBCount++;
  comparisonFlag = 0;
}

void encoderRGCounter( void )
{
  encoderRGCount++;
  comparisonFlag = 0;
}

//A*
void AStar( void)
{
  /*
  A* Pathfinding
  A rundown of how the algorithm works can be found in the final report.
  The A* function first computes the shortest route through the maze as descripted in the report. Once the goal has been found the 
  cost assigning stops and the parent squares are then saved in a list from the goal back to the start. 
  The algorithm then uses this list of coordinates and converts them into into a set of movement instructions as it moves from the start to the end
  Once it reaches the end the robot stops and exits the Algorithm.
  */
  // First all of the data structures are initilized. all of the A*'s structures are local, with the exception of the shortest path vector which is global
  byte i = 0;
  byte j = 0;

  boolean found = 0;
  boolean openList[6][6] = {0};
  boolean closedList[6][6] = {0};
  byte costs[6][6][6] = {0}; // 0 = F cost, 1 = G cost, 2 = H cost, 3 = Parent X coord, 4 = Parent Y coord, 5 = # of parents

  byte bestX = 255;
  byte bestY = 255;
  byte tempX = 255;
  byte tempY = 255;

  // All the costs of all the squares are set to themaximum a byte allows so if accidentally added to the open list they still still not get searched
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      costs[i][j][0] = 255;
    }
  }
  Serial.println("Starting A*");
  //Set initial condidtions - the search ends when the goal square enters the openList so it's initialised to a high F cost to be used in the first interation of the loop
  openList[0][0] = 1;
  costs[0][0][0] = 0;
  costs[5][5][0] = 255;
  Serial.println("Initial conditions done");
  while (!found)
  {
    bestX = 5; // Every loop set the initial lowest cost square to the goal, It is the only square that remains at maximum cost for entire duration of the search.
    bestY = 5;
    // Search open list for lowest F cost, then select it and put it in closed list
    for (i = 0; i < 6; i++)
    {
      for (j = 0; j < 6; j++)
      {
        if (openList[i][j] == 1)
        {
          if (costs[i][j][0] < costs[bestX][bestY][0]) // if the selected square is both a lower cost than the curret selected square and in the open list, make it the current square
          {
            bestX = i;
            bestY = j;
          }
        }
      }
    }
    Serial.print("Found lowest F cost: "); // Print the square selected as the lowest cost as debugging info
    Serial.println(costs[bestX][bestY][0]);
    Serial.print("Square: x: ");
    Serial.print(bestX);
    Serial.print(" y: ");
    Serial.println(bestY);
    
    closedList[bestX][bestY] = 1; // add the square to the closed list so that it does not get searched again.
    openList[bestX][bestY] = 0; // Remove the square from the open list for the same reasons
    // Calculate adjacent squares cost (if it is a valid move or not already closed), if cost is lower than it's current cost update cost and make selected square it's parent
    if ((edgeMatrix[bestX][bestY][north] == 1) && ( closedList[bestX + 1][bestY] == 0))
    {
      costs[bestX + 1][bestY][1] = costs[bestX][bestY][1] + 1; // Calculate G cost
      costs[bestX + 1][bestY][2] = (5 - (bestX + 1)) + (5 - bestY); // Estimate H cost
      costs[bestX + 1][bestY][0] = costs[bestX + 1][bestY][1] + costs[bestX + 1][bestY][2]; // Calculate F cost
      costs[bestX + 1][bestY][3] = bestX; // Save parent x-coord
      costs[bestX + 1][bestY][4] = bestY; // Save parent y-coord
      costs[bestX + 1][bestY][5] = costs[bestX][bestY][5] + 1; // Update # of parent squares
      openList[bestX + 1][bestY] = 1; // Add square to open list
    }

    if ((edgeMatrix[bestX][bestY][south] == 1) && ( closedList[bestX - 1][bestY] == 0))
    {
      costs[bestX - 1][bestY][1] = costs[bestX][bestY][1] + 1; // Calculate G cost
      costs[bestX - 1][bestY][2] = (5 - (bestX - 1)) + (5 - bestY); // Estimate H cost
      costs[bestX - 1][bestY][0] = costs[bestX - 1][bestY][1] + costs[bestX - 1][bestY][2]; // Calculate F cost
      costs[bestX - 1][bestY][3] = bestX; // Save parent x-coord
      costs[bestX - 1][bestY][4] = bestY; // Save parent y-coord
      costs[bestX - 1][bestY][5] = costs[bestX][bestY][5] + 1; // Update # of parent squares
      openList[bestX - 1][bestY] = 1; // Add square to open list
    }

    if ((edgeMatrix[bestX][bestY][east] == 1) && ( closedList[bestX][bestY + 1] == 0))
    {
      costs[bestX][bestY + 1][1] = costs[bestX][bestY][1] + 1; // Calculate G cost
      costs[bestX][bestY + 1][2] = (5 - bestX) + (5 - (bestY + 1)); // Estimate H cost
      costs[bestX][bestY + 1][0] = costs[bestX][bestY + 1][1] + costs[bestX][bestY][2]; // Calculate F cost
      costs[bestX][bestY + 1][3] = bestX; // Save parent x-coord
      costs[bestX][bestY + 1][4] = bestY; // Save parent y-coord
      costs[bestX][bestY + 1][5] = costs[bestX][bestY][5] + 1; // Update # of parent squares
      openList[bestX][bestY + 1] = 1; // Add square to open list
    }

    if ((edgeMatrix[bestX][bestY][west] == 1) && ( closedList[bestX][bestY - 1] == 0))
    {
      costs[bestX][bestY - 1][1] = costs[bestX][bestY][1] + 1; // Calculate G cost
      costs[bestX][bestY - 1][2] = (5 - bestX) + (5 - (bestY - 1)); // Estimate H cost
      costs[bestX][bestY - 1][0] = costs[bestX][bestY - 1][1] + costs[bestX][bestY][2]; // Calculate F cost
      costs[bestX][bestY - 1][3] = bestX; // Save parent x-coord
      costs[bestX][bestY - 1][4] = bestY; // Save parent y-coord
      costs[bestX][bestY - 1][5] = costs[bestX][bestY][5] + 1; // Update # of parent squares
      openList[bestX][bestY - 1] = 1; // Add square to open list
    }
    Serial.println("Scored adjacents");
    // Check if goal is in open list and set loop condition false, goal has been found and the shortes route can be found
    if (openList[5][5] == 1)
    {
      found = 1;
    }
  }
  Serial.println("Calculated Costs, finding fastest route");
  // Calculate shortest path by following the parent squares backwards from goal
  bestX = 5;
  bestY = 5;
  Serial.print("# of parents goal has: ");
  Serial.println(costs[0][0][5]);
  for (i = costs[5][5][5]; i > 0; i--)
  {
    shortestPath[i][0] = bestX;
    shortestPath[i][1] = bestY;
    tempX = costs[bestX][bestY][3];
    tempY = costs[bestX][bestY][4];
    bestX = tempX;
    bestY = tempY;
  }
  Serial.println("Found fastest route, printing..."); // Once the shortest path is saved to shortestPath, print it for debugging
  for(i = 0; i < 36; i++)
  {
    Serial.print("X: ");
    Serial.print(shortestPath[i][0]);
    Serial.print(" Y: ");
    Serial.println(shortestPath[i][1]);
  }

  /*
  Drive to goal
  The next section works exactly as the backtracking works in DFA()
  */
  int findX = 0;
  int findY = 0;
  x = 0; // Set start point to square (0,0)
  y = 0;
  i = 0; // Reset counter
  orientation = north; // Make sure robot knows it's facing north, It should already be but again just incase 

  while (i < 35)

  {
    if(x == 5)
    {
      if(y == 5)
      {
        break; // If the robot is in square (5,5) the goal, break otu of the infinite loop
      }
    }
    // Only one coordinate can change every square moved, therefore to find the direction
      // to travel the coordinated to move to are subtracted from the current coordinates.
      // This then gives the direction to travel as follows.
      // X1 - X2 = -1 --> move north
      // x1 - x2 = 1  --> move south
      // y1 - y2 = 1  --> move west
      // y1 - y2 = -1 --> move east
    findX = x - shortestPath[i][0]; // calculate x co-ord difference
    findY = y - shortestPath[i][1]; // Calculate y co-ord difference

    Serial.print("findX: "); // Print debugging code for where it is going
    Serial.println(findX);
    Serial.print("findY: ");
    Serial.println(findY);
    
    if (findX == -1) // Move north
    {
      turn(north);
      orientation = north;
      forward(1);
      x = shortestPath[i][0]; // Set the next x co-ord in the shortest path list to the current square
      y = shortestPath[i][1]; // Set the next y co-ord in the shortest path list to the current square
    }
    else if (findX == 1) // Move south
    {
      turn(south);
      orientation = south;
      forward(1);
      x = shortestPath[i][0]; // Set the next x co-ord in the shortest path list to the current square
      y = shortestPath[i][1]; // Set the next y co-ord in the shortest path list to the current square
    }
    else if (findY == -1) // Move east
    {
      turn(east);
      orientation = east;
      forward(1);
      x = shortestPath[i][0]; // Set the next x co-ord in the shortest path list to the current square
      y = shortestPath[i][1]; // Set the next y co-ord in the shortest path list to the current square
    }
    else if (findY == 1) // Move west
    {
      turn(west);
      orientation = west;
      forward(1);
      x = shortestPath[i][0]; // Set the next x co-ord in the shortest path list to the current square
      y = shortestPath[i][1]; // Set the next y co-ord in the shortest path list to the current square
    }
    else; // It should never not have to move but this is here for redundancy
    
    i++; // increment shortestPath index
  }
}
