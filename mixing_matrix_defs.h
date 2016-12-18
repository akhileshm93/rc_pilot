/*******************************************************************************
* mixing_matrix_defs.h
*
* James Strawson 2016
* matrix definitions for mixing_matrix.c
*
*  				rotor
*		X Y Z  Roll Pitch Yaw
* 	1
*  	2
* 	3
*   4
*	5
*	6
* Z points down so positive yaw is turning right from 1-st person p
*******************************************************************************/

#define MAX_INPUTS 6	// up to 6 control inputs (r,p,yaw,z,x,y)	
#define MAX_ROTORS 8	// up to 8 rotors



// my 6D0F control
//  6  1       cw ccw      	X
// 5    2    ccw     cw    	^
//  4  3       cw ccw      	+ > Y
//   X          Y         Z         Roll      Pitch       Yaw
double mix_rotorbits_6dof[][6] = {\
{-0.2736,    0.3638,   -1.0000,   -0.2293,    0.3921,    0.3443},\
{ 0.6362,    0.0186,   -1.0000,   -0.3638,   -0.0297,   -0.3638},\
{-0.3382,   -0.3533,   -1.0000,   -0.3320,   -0.3638,    0.3546},\
{-0.3382,    0.3533,   -1.0000,    0.3320,   -0.3638,   -0.3546},\
{ 0.6362,   -0.0186,   -1.0000,    0.3638,   -0.0297,    0.3638},\
{-0.2736,   -0.3638,   -1.0000,    0.2293,    0.3921,   -0.3443}\
};



// Most popular: 4-rotor X layout like DJI Phantom and 3DR Iris
//  4   1       cw ccw      X   Z down
//    X     			   	^
//  3   2       ccw cw      + > Y
// X     Y      Z     Roll  Pitch   Yaw
double mix_4x[][6] = {\
{0.0,   0.0,  -1.0,  -0.5,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.5,  -0.5,  -0.5},\
{0.0,   0.0,  -1.0,   0.5,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.5,   0.5,  -0.5}\
};


// less popular: 4-rotor + layout
//    1       ccw      	X   Z down
//  4 + 2   cw   cw		^
//    3       ccw      	+ > Y
// X     Y     Z     Roll  Pitch   Yaw
double mix_4plus[][6] = {\
{0.0,   0.0,  -1.0,   0.0,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.5,   0.0,  -0.5},\
{0.0,   0.0,  -1.0,   0.0,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.5,   0.0,  -0.5}\
};

// 6X like DJI S800
//  6  1       cw ccw      	X
// 5    2    ccw     cw    	^
//  4  3       cw ccw      	o > Y
// X     Y      Z     Roll   Pitch   Yaw
double mix_6x[][6] = {\
{0.0,   0.0,  -1.0,  -0.25,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.50,   0.0,  -0.5},\
{0.0,   0.0,  -1.0,  -0.25,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.25,  -0.5,  -0.5},\
{0.0,   0.0,  -1.0,   0.50,   0.0,   0.5},\
{0.0,   0.0,  -1.0,   0.25,   0.5,  -0.5}\
};


// 8X like DJI S1000
//	 8 1		   cw ccw
// 7     2       ccw     cw    	X
// 6     3    	 cw     ccw    	^
//   5 4       	   ccw cw      	o > Y
// X     Y      Z     Roll    Pitch   Yaw
double mix_8x[][6] = {\
{0.0,   0.0,  -1.0,  -0.21,   0.50,   0.5},\
{0.0,   0.0,  -1.0,  -0.50,   0.21,  -0.5},\
{0.0,   0.0,  -1.0,  -0.50,  -0.21,   0.5},\
{0.0,   0.0,  -1.0,  -0.21,  -0.50,  -0.5},\
{0.0,   0.0,  -1.0,   0.21,  -0.50,   0.5},\
{0.0,   0.0,  -1.0,   0.50,  -0.21,  -0.5},\
{0.0,   0.0,  -1.0,   0.50,   0.21,   0.5},\
{0.0,   0.0,  -1.0,   0.21,   0.50,  -0.5}\
};


 