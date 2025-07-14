/*
 * its important, read it!!
 * the only two variable are there that you have to change in the drawline() function in last four if blocks
 * the first oen is the value (12.7) which is actually C/2 where C is the distance b/w both the servos (x and y one) in my case its 25.4 so the variable 12.7
 * which needs to be be change acc to your plotter 
 * 
 * and the other one is the value 117 which is the total distance from origin and the servos moddle point, in my case 72mm from the end of writing area and 45mm is the y axis max so total of 127
 * change this value acc to your need.
 */




#include <Servo.h>

Servo penservo;
Servo xservo;    //xservo mean the servo on left
Servo yservo;    // yservo mean the right one
 

#define LINE_BUFFER_LENGTH 512

// pen lift servo angles
const int penZUp = 23; 
const int penZDown = 40;


/* global variables   */
struct point { 
  float x; 
  float y; 
  float z; 
};

struct point actuatorPos;

// drawing settings  
float StepInc = 1;
int StepDelay = 0;
int LineDelay = 50;
int penDelay = 50;


const float v = 57.295;  // v var acc  is 180/pie(3.14) to convert radian in to degree

// writing area
float Xmax =60;
float Xmin =-60;
float Ymax =45;
float Ymin =-45;

boolean verbose = false;


void setup() {

  Serial.begin( 9600 );
  Serial.begin(9600);
  // initialising all the servos
  penservo.attach(6);
  xservo.attach(3);
  yservo.attach(9);

  Serial.print("X goes from "); 
  Serial.print(Xmin); 
  Serial.print(" to "); 
  Serial.print(Xmax); 
  Serial.println(" mm."); 
  Serial.print("Y goes from "); 
  Serial.print(Ymin); 
  Serial.print(" to "); 
  Serial.print(Ymax); 
  Serial.println(" mm."); 

  penUp();
  delay(500);
  xservo.write(90);
  yservo.write(90);
  delay(500);
  
}

void loop() 
{

  //its just a simple gcode interpretor
  
  delay(200);
  char line[ LINE_BUFFER_LENGTH ];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  while (1) {
    while ( Serial.available()>0 ) {
      c = Serial.read();
      if (( c == '\n') || (c == '\r') ) {       
        if ( lineIndex > 0 ) {                        
          line[ lineIndex ] = '\0';                   
          if (verbose) { 
            Serial.print( " Received: "); 
            Serial.println( line ); 
          }
          processIncomingLine( line, lineIndex );
          lineIndex = 0;
        } 
        else { 
          //Empty comment line, skip block.  
        }
        lineIsComment = false;
        lineSemiColon = false;
        Serial.println("ok");    
      } 
      else {
        if ( (lineIsComment) || (lineSemiColon) ) {   
          if ( c == ')' )  lineIsComment = false;     
        } 
        else {
          if ( c <= ' ' ) {                           
          } 
          else if ( c == '/' ) {                    
          } 
          else if ( c == '(' ) {                    
            lineIsComment = true;
          } 
          else if ( c == ';' ) {
            lineSemiColon = true;
          } 
          else if ( lineIndex >= LINE_BUFFER_LENGTH-1 ) {
            Serial.println( "ERROR - buffer overflow" );
            lineIsComment = false;
            lineSemiColon = false;
          } 
          else if ( c >= 'a' && c <= 'z' ) {        
            line[ lineIndex++ ] = c-'a'+'A';
          } 
          else {
            line[ lineIndex++ ] = c;
          }
        }
      }
    }
  }
}

void processIncomingLine( char* line, int charNB ) {
  int currentIndex = 0;
  char buffer[ 64 ];                               
  struct point newPos;

  newPos.x = 0.0;
  newPos.y = 0.0;
  
  while( currentIndex < charNB ) {
    switch ( line[ currentIndex++ ] ) {             
    case 'U':
      penUp(); 
      break;
    case 'D':
      penDown(); 
      break;
    case 'G':
      buffer[0] = line[ currentIndex++ ];         
      buffer[1] = '\0';

      switch ( atoi( buffer ) ){                   // working with g command
      case 0:                                   //no need to diff b/w move or move faster 
      break;
      case 1:
        // imp:: Here we have assume that the gcode string received have X vaiue before Y
        char* indexX = strchr( line+currentIndex, 'X' ); 
        char* indexY = strchr( line+currentIndex, 'Y' );
        if ( indexY <= 0 ) {
          newPos.x = atof( indexX + 1); 
          newPos.y = actuatorPos.y;
        } 
        else if ( indexX <= 0 ) {
          newPos.y = atof( indexY + 1);
          newPos.x = actuatorPos.x;
        } 
        else {
          newPos.y = atof( indexY + 1);
          indexY = '\0';
          newPos.x = atof( indexX + 1);
        }
        drawLine(newPos.x, newPos.y );
   
        actuatorPos.x = newPos.x;
        actuatorPos.y = newPos.y;
        break;
      }
      break;
    case 'M':
      buffer[0] = line[ currentIndex++ ];        // /!\ M commands must not increase 3-digit
      buffer[1] = line[ currentIndex++ ];
      buffer[2] = line[ currentIndex++ ];
      buffer[3] = '\0';
      break;
    case 'A':
      // Handle ANGLES command for direct servo angle control
      if (line[currentIndex] == 'N' && 
          line[currentIndex+1] == 'G' && 
          line[currentIndex+2] == 'L' && 
          line[currentIndex+3] == 'E' && 
          line[currentIndex+4] == 'S') {
            
        currentIndex += 5;  // Skip over "NGLES"
        
        // Find the space after ANGLES
        while (line[currentIndex] == ' ' && currentIndex < charNB) {
          currentIndex++;
        }
        
        // Parse first angle (alpha)
        float alpha = 0.0;
        char angleBuffer[10] = {0};
        int i = 0;
        
        while (line[currentIndex] != ' ' && currentIndex < charNB && i < 9) {
          angleBuffer[i++] = line[currentIndex++];
        }
        angleBuffer[i] = '\0';
        alpha = atof(angleBuffer);
        
        // Skip spaces between angles
        while (line[currentIndex] == ' ' && currentIndex < charNB) {
          currentIndex++;
        }
        
        // Parse second angle (beta)
        float beta = 0.0;
        i = 0;
        memset(angleBuffer, 0, 10);
        
        while (line[currentIndex] != ' ' && currentIndex < charNB && i < 9) {
          angleBuffer[i++] = line[currentIndex++];
        }
        angleBuffer[i] = '\0';
        beta = atof(angleBuffer);
        
        // Apply servo angles directly
        servowrite(alpha, beta);
        
        if (verbose) {
          Serial.print("Direct angles set - Alpha: ");
          Serial.print(alpha);
          Serial.print(", Beta: ");
          Serial.println(beta);
        }
      }
      break;
    }
  }
}


void drawLine(float x1, float y1){
  
  if (verbose){
    Serial.print("fx1, fy1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }  

  // setting limits for each axis
  //(x1,y1) are the points where the pen have to reach, fetched from the comming gcode strings
  if (x1 >= Xmax) { 
    x1 = Xmax; 
  }
  if (x1 <= Xmin) { 
    x1 = Xmin; 
  }
  if (y1 >= Ymax) { 
    y1 = Ymax; 
  }
  if (y1 <= Ymin) { 
    y1 = Ymin; 
  }

  if(x1 < -12.7){
    float tx1= atan((abs(x1)-12.7)/(117-y1));
    float dx1= sqrt(sq(abs(x1)-12.7) + sq(117-y1));
    float cx1= acos(dx1/180);
    float anglex1= 135 - v*(tx1 + cx1);

    float ty1= atan((abs(x1)+12.7)/(117-y1));
    float dy1= sqrt(sq(abs(x1)+12.7) + sq(117-y1));
    float cy1= acos(dy1/180);
    float angley1= 45 + v*(cy1-ty1); 

    servowrite(anglex1,angley1);
 }

  else if(x1 >= -12.7 && x1<0){
    
    float tx1= atan((12.7-abs(x1))/(117-y1));
    float dx1= sqrt(sq(12.7-abs(x1)) + sq(117-y1));
    float cx1= acos(dx1/180);
    float anglex1= 135 - v*(cx1-tx1);

    float ty1= atan((abs(x1)+12.7)/(117-y1));
    float dy1= sqrt(sq(abs(x1)+12.7) + sq(117-y1));
    float cy1= acos(dy1/180);
    float angley1= 45 + v*(cy1-ty1); 

    servowrite(anglex1,angley1);
    
  }

  else if(x1 >=0 && x1 < 12.7){
    
    float tx1= atan((x1+12.7)/(117-y1));
    float dx1= sqrt(sq(12.7+x1) + sq(117-y1));
    float cx1= acos(dx1/180);
    float anglex1= 135 - v*(cx1-tx1);

    float ty1= atan((12.7-x1)/(117-y1));
    float dy1= sqrt(sq(12.7-x1) + sq(117-y1));
    float cy1= acos(dy1/180);
    float angley1= 45 + v*(cy1-ty1); 

    servowrite(anglex1,angley1);
    
  }

  else if( x1 >= 12.7){
    
    float tx1= atan((x1+12.7)/(117-y1));
    float dx1= sqrt(sq(x1+12.7) + sq(117-y1));
    float cx1= acos(dx1/180);
    float anglex1= 135 - v*(cx1-tx1);

    float ty1= atan((x1-12.7)/(117-y1));
    float dy1= sqrt(sq(x1-12.7) + sq(117-y1));
    float cy1= acos(dy1/180);
    float angley1= 45 + v*(cy1+ty1); 

    servowrite(anglex1,angley1);
    
  }
}


void servowrite(float a, float b){
  xservo.write(lroundf(a));
  yservo.write(lroundf(b));
}



void penUp(){ 
  if(penservo.read()!= penZUp){
    penservo.write(penZDown-10);
    for(int i=0; i<=7; i++){
      penservo.write(penZDown-10-i);
      delay(50);
    }
  }
}


void penDown() {
    if(penservo.read()!= penZDown){
    for(int j=0; j<=7; j++){
      penservo.write(penZUp+j);
      delay(50);
    }
    penservo.write(penZDown);
  }
}

