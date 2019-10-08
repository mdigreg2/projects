/*
AUTHOR: Michael DiGregorio
I pledge my honor that I have abided by the Stevens Honor System
*/



//make an animated circle that follows the edges of the screen

void setup() {
 size(500,500); 
  
}

 int dx = 5;
 int dy = 5;
 float speed = 5;
 int r = 20;
 int x = 10;
 int y = 10;
 int xLim = (width-20)/dx;
 int yLim = (height-20)/dy;
 boolean topLeft = false;
 boolean topRight = false;
 boolean bottomRight = false;
 boolean bottomLeft = false;
 Ball ball = new Ball(x, y, speed, r);
void draw(){
  background(200,200,255);
  ball.drawBall();
  if(ball.xPos == 10 && ball.yPos == 10){
    topLeft = true;
    topRight = false;
    bottomRight = false;
    bottomLeft = false;
  }
  if(topLeft == true){
   ball.moveRight(); 
  }
  if(ball.xPos >= 489 && ball.yPos <= 10){
    topLeft = false;
    topRight = true; 
    bottomRight = false;
    bottomLeft = false;
  }
  if(topRight == true){
   ball.moveDown(); 
  }
  if(ball.xPos >= 489 && ball.yPos >= 489){
   topLeft = false;
   topRight = false;
   bottomRight = true;
   bottomLeft = false;
  }
  if(bottomRight == true){
   ball.moveLeft(); 
  }
  if(ball.xPos <= 10 && ball.yPos >= 489){
   topLeft = false;
   topRight = false;
   bottomRight = false;
   bottomLeft = true;
  }
  if(bottomLeft == true){
   ball.moveUp(); 
  }
}




 //  //background(100,100,255);
 // for(int i = 0; i < 96; i++){
 //  circle(x, y, r);
 //  x += dx;
 // println(x);
 //  //background(100,100,255);
 // }
 // //background(100,100,255);
 // for(int i = 0; i < 96; i++){
 //  circle(x, y, r);
 //  y += dy;
 //println(x);
 // }
 //  //background(100,100,255);
 // for(int i = 0; i < 96; i++){
 //  circle(x, y, r);
 //  x -= dx;
 //println(x);
 // }
 //  //background(100,100,255);
 // for(int i = 0; i < 96; i++){
 //  circle(x, y, r);
 //  y -= dy;
 //  println(x);
 // }
//}









//int r = 40;
//int x = r / 2;
//int y = r / 2;
//float s = 5;
//Ball ball = new Ball(x, y, s, r);
//boolean top = true;
//boolean right = false;
//boolean bottom = false;
//boolean left = false;
//int xLim = (width - r/2);
//int yLim = (height - r/2);
//void draw() {
//  //background(200,200,255);
//  ball.drawBall();
//  for (float i = r/2; i <= xLim; i+= s){
//  // ball.drawBall();
//   ball.moveRight();
//  }
//  println(ball.xPos);
//}// draw



//if ((x < width -10) && top==true){
// for(int i = 0; i < 96; i += 1){
//  circle(x, y, r);
//  x += dx;
// }
//}
//if((x >= width - 10)){
//   top = false;
//  for(int i = 0; i < 96; i += 1){
//   circle(x, y, r);
//   y += dy;
// }
//}
//if((x>=width-10) && top == false){
// for(int i = 0; i < 97; i += 1){
//  circle(x, y, r);
//  x -= dx;
// }
//}
//if((x<=width-10) && top == false){
// for(int i = 0; i < 97; i += 1){
//  circle(x, y, r);
//  y -= dy;
// }
//}
// top = true;
  

//int xPos = 40, yPos = 40;
//int CIRCLE_RADIUS = 80;

//void draw() {
// setAttribs();
// circle(xPos,yPos,CIRCLE_RADIUS); 
 
// if(xPos < width - CIRCLE_RADIUS) {
//  moveRight(xPos, 1); 
// }
  
//}

//void setAttribs() {
// stroke(0,0,200);
// strokeWeight(5);
// fill(150,150,255);
//}


//void moveRight(int x, int s){
//  x += s;
//}
//void moveLeft(int x, int s){
//  moveRight(x, -s);
//}
//void moveDown(int y, int s){
  
//}
//void moveUp(int y, int s){
//  moveDown(y, -s);
//}
