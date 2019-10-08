class Ball{
 int xPos;
 int yPos;
 float speed;
 int radius;
 
 Ball(int x, int y, float s, int r) {
  xPos = x;
  yPos = y;
  speed = s;
  radius = r;
 }
 void drawBall(){
  circle(xPos, yPos, radius); 
 }
 void moveRight(){
   this.xPos += this.speed;
 }
 void moveLeft(){
  this.xPos -= this.speed; 
 }
 void moveUp(){
  this.yPos -= this.speed; 
 }
 void moveDown(){
  this.yPos += this.speed; 
 }
  
}
