/****************************************************************************
* Author: Team Robotec
* Date: October 30th, 2012
* Version: 1.0
*
* Software for maze solving robot.
* This software takes left path as first priority, right path as second 
* priority, and straight as lowest priority.
* Path memorization is done by keeping count of turns made and turns not 
* made in a back-tracking manner.
****************************************************************************/

// Includes
#include "SoR_Utils.h" //includes all the technical stuff
#include <math.h> // Used for pow() to convert voltage reading to distance

// Sensor port mapping
#define irSensor 5

// Predefined distance for paths in cm
#define leftPathDis 15
#define rightPathDis 15
#define frontPathDis 15 
#define minFrontDis 12


//Sensor ranges
#define minDis 5 // Tested minimum is 5, but maybe add some margin

// Marco to take sensor readings from left, straight and right
#define takeAllReadings(startPst) sensorToLeft(startPst);sensorToStraight(startPst);sensorToRight(startPst)
// Macro to make left turn
#define makeLeft(start, delay) turnLeft(start);delay_cycles(delay);straightBlock(start)
// Marco to make right turn
#define makeRight(start, delay) turnRight(start);delay_cycles(delay);straightBlock(start)  
 
  	/*** GLOBAL VARIABLES ***/
	/*****************Path Memorization Variables*******/
	int leftPath, rightPath, straightPath;
	/*****************Sensor Values Variables***********/
	int frontIR=0,//straight range
		leftIR=0, //left range
		rightIR=0, //left range
		leftTurnCount = 0; //left turn count for maze completion detection

// Convert short-range analog infrared values to real-world (centimeter [cm]) values
int shortIR(int value){
	return 509.88*pow(value,-.9154);
}

// Set sensor to point straight and take reading
void sensorToStraight(int i){
	while( i < 80){
		servo_scan(41);
		i++;
	}
	delay_cycles(5000);
	i = 0;
	while( i < 100){
		frontIR = shortIR(a2dConvert8bit(irSensor));
		i++;
	}
	delay_cycles(5000);
	if (frontIR > frontPathDis || frontIR < ( minDis))
		straightPath = 1;
    else
        straightPath = 0;
}

// Set sensor to right and take reading
void sensorToRight(int i ){
	while( i < 80){
		servo_scan(68);
		i++;
	}
	delay_cycles(5000);
	i = 0;
	while( i < 100){
		rightIR = shortIR(a2dConvert8bit(irSensor));
		i++;
	}
	delay_cycles(5000);
	if (rightIR > rightPathDis || rightIR < ( minDis))
      rightPath = 10;
	else
      rightPath = 0;
}

// Set sensor to left and take reading
void sensorToLeft(int i){
	while( i < 80){
		servo_scan(17);
		i++;
	}
	delay_cycles(5000);
	i = 0;
	while( i < 100){
		leftIR = shortIR(a2dConvert8bit(irSensor));
		i++;
	}
	delay_cycles(5000);
   if (leftIR > leftPathDis || leftIR < ( minDis))
      leftPath = 100;
   else
      leftPath = 0;	
}

// Move forward in 7 inch block
void straightBlock(int i){
	while( i < 1){
		servo_left(26);
		i++;
	}
	delay_cycles(20000);
	i = 0;
	while(i < 33){
		if(i%5==0){
			servo_right(39);
			servo_left(32);
		}
		else{
			servo_right(38);
			servo_left(32);
		}

		i++;
	}
	delay_cycles(20000);
	servo_left(25);
}

// Rotate left approximately 90 degrees
void turnLeft(int i){
	while(i < 13){
		servo_right(44);
		servo_left(43);
		i++;
	}
	delay_cycles(20000);
}

// Rotate right approximately 90 degrees
void turnRight(int i){
	while(i < 13){
		servo_right(27);
		servo_left(27);
		i++;
	}
	delay_cycles(20000);
}

// Rotate approximately 180 degrees
void rotate180(void){
	turnLeft(0);
	delay_cycles(20000);
	turnLeft(0);
	straightBlock(0);
}

// Needed when backtracking from something like left path to right path, but there's also straight path in the middle to bypass
void bypassSection(void){ 
    if( leftPath > 0 || rightPath > 0){ // Ignore and bypass side turns
		straightBlock(0);
	}
}

int main(void)
	{
	//declare variables here
	/*****************Path Memorization Variables*******/
	int isBacktracking = 0; // Boolean for backtracking mode
	int turnCount = 0; // number of turns made
	int turnMade [10]; // array to store all the turns made -- used to find overall solution and when backtracking
	int turnNotMade [10]; // array to store all the turns not made -- used and converted to turnMade when backtracking
	/***************************************************/



	/****************INITIALIZATIONS*******************/
	configure_ports(); // configure which ports are analog, digital, etc.
	a2dInit(); // initialize analog to digital converter (ADC)
	a2dSetPrescaler(ADC_PRESCALE_DIV32); // configure ADC scaling
	a2dSetReference(ADC_REFERENCE_AVCC); // configure ADC reference voltage
	/**************************************************/
	
	/*********Maze Path Chosing Code**********/
	// Delay start up and flash the LED on-off 5 times
	for(int i = 0; i <5; i++){
		LED_on();
		delay_cycles(14000);
		LED_off();
		delay_cycles(14000);
	}

	LED_on(); // Turn LED back on
	
	while(1){
		// Store distance from walls by using return voltage from sensor
		takeAllReadings(0);

		// If four consecutive turns have been made, the maze is solved. Breaks out of loop.
		if (leftTurnCount == 4)
			break;
         /* Path Choosing & Memorization */
        if (isBacktracking == 0){ // We're not backtracking yet (We haven't traveled here before)
			if(leftPath == 100){ // Turn left if no wall on left side (left path takes priority over all paths)
            //TURN_LEFT_WITH_BUFFER(50,0,0);
            makeLeft(0,5000);
            	leftTurnCount++; // Keep track of left turns for maze completion
				turnMade[turnCount] = leftPath; //Memorize the turns just made
				turnNotMade[turnCount] = rightPath + straightPath; // Turns need to make later
				turnCount++; /* Only increment when actual turns are made */
			}
			else if(rightPath == 10){ // Turn right if wall on left side, but no wall on right
				turnMade[turnCount] = rightPath;
				turnNotMade[turnCount] = straightPath;
				turnCount++;
            	leftTurnCount = 0; // Since turn was not left, reset counter.
            	makeRight(0,5000);
            }else{
			 	leftTurnCount = 0; // Since turn was not left, reset counter.
				if(frontIR > minFrontDis /*consecutiveDeadEndConfLeft>0*/){ // Go straight if there are walls on both sides and there's no dead-end close ahead [this "I see dead-end" needs to be reported 3 times to be believed, in order to avoid trusting false readings]	
					straightBlock(0);
				}else{ // Turn around if dead-end and walls on both sides (no more path available)
				    rotate180(); // Rotate 180 degrees
				    isBacktracking=1; // We're backtracking now since there's no more path around us here
				}
			}
		}/********* End Maze Path Choosing (withOUT backtracking) **********/
       else{ // We're backtracking now
          // If any, do return trip/backtrack to turns recorded in turnNotMade, so we can make those turns and convert them into turnMade
			 if (leftPath > 0 || rightPath > 0){ // Check if running into an intersection (has left/right path available now)
		   	 	turnCount--; // Look at the previous turn made/not

				if (turnMade[turnCount] == 100){      // If left turn was made
			 		if (turnNotMade[turnCount] > 1) {  // Right turn still needed to be made, so turn right this time [which is going straight in current view]
						turnMade[turnCount] = 10; //rightPath;     // Remember we have made this turn now
						turnNotMade[turnCount] -= 10; //rightPath; // Take this turn out of notMade
						isBacktracking = 0;
						turnCount++;                  // We're done and not backtracking anymore
                        bypassSection();
					} else if (turnNotMade[turnCount] == 1) {  // Going straight still needed to be made (but done/not need right turn), so go straight [which is turning left in current view]
						turnMade[turnCount] = 1; //straightPath;
						turnNotMade[turnCount] -= 1; //straightPath;
						turnCount++;
						isBacktracking = 0;
                  		//TURN_LEFT_WITH_BUFFER(50,0,0);
                  		makeLeft(0,5000);
                  
					} else { // We have made all turns of the current intersection already
						turnMade[turnCount] = 0;
						turnCount++;
                  		//TURN_RIGHT_WITH_BUFFER(50,0,0); // Turn back to the previous section again [which is turning right in current view]
                  		makeRight(0,5000);
                  
					}
				}else if (turnMade[turnCount] == 10){      // If right turn was made [We don't worry about left turn at this point]
					if (turnNotMade[turnCount] == 1) {       // Going straight still needed to be made, so go straight [which is turning right in current view]                  
						turnMade[turnCount] = 1; //straightPath;
						turnNotMade[turnCount] -= 1; //straightPath;
						isBacktracking = 0;
                  
                  //TURN_RIGHT_WITH_BUFFER(50,0,0); // Turn back to the previous section again [which is turning right in current view]
                  makeRight(0,20000);
                  
					}else { // We have made all turns of the current intersection already
						turnMade[turnCount] = 0;
						
						//TURN_LEFT_WITH_BUFFER(50,0,0); // Turn back to the previous section again [which is turning left in current view]
						makeLeft(0,20000);
                  
					}
				}else { // We kept going straight (turnMade[turnCount] == 1), which means we should not make any turns in the current intersection and just pick and keep going straight
					 turnMade[turnCount] = 0;
                        bypassSection();
				}               
			}else{ // No intersection found
				straightBlock(0);//goStraight(); // Just keep going straight until we find the previous intersection we need to backtrack to
			}
		} /********* End Maze Path Choosing (WITH backtracking) **********/
      // A small delay to prevent crazy oscillations
		delay_cycles(200);		
	}
	// Maze completed, flash led.
	while(1){
		LED_off();
		delay_cycles(7000);
		LED_on();
		delay_cycles(10000);
	}
	return 0;
}
