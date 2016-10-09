#ifndef MyTypes_h
#define MyTypes_h

typedef struct {
  float xpos;
  float ypos;
  float zpos;
  float xtarget;
  float ytarget;
  float ztarget;
  int xangle;
  int yangle;
  int zangle;
  int xcarry;
  int ycarry;
  int zcarry;
} location_st;

//xpos is position
//xangle is the angle
//xcarry is legacy and should be removed

#endif 

