/*
AUTHOR: Michael DiGregorio
I pledge my honor that I have abided by the Stevens Honor System
*/

//draw a sierpinski triangle to the level indicated


void setup(){
  size(600,600, P3D);  
  frameRate(1.5);
}

void sierpinski(float x1, float y1, float x2, float y2, float x3, float y3, int level){
  if(level == 0){
     //draw the triangle and leave
     beginShape();
     fill(255,150,150);
     vertex(x1,y1);
     fill(150,150,255);
     vertex(x2,y2);
     fill(150,255,150);
     vertex(x3,y3);
     endShape();
     //triangle(x1,y1, x2,y2, x3,y3);
     return;
  }
  else{
    float xa = (x1+x2)/2;
    float xb = (x1+x3)/2;
    float xc = (x3+x2)/2;
    float ya = (y1+y2)/2;
    float yb = (y1+y3)/2;
    float yc = (y3+y2)/2;
   //draw three other triangles
   sierpinski(x1,y1, xa,ya, xb,yb, level-1);
   sierpinski(xa,ya, x2,y2, xc,yc,level-1);
   sierpinski(xb,yb, xc,yc, x3,y3, level-1);
  }
} 
int counter = 0;
//boolean ascending = true;
void draw(){
  background(50);
  float a = height-1;
  float b = width/2;
  float c = width -1;
  float d = height-1;
  sierpinski(0.0,a, b,1.0, c,d, counter);
  if(counter < 6)
    counter ++;
  else 
    counter = 0;
  //if (counter > 6)
  //  ascending = false;
  //if (counter < 0)
  //  ascending = true;
  //if (ascending)
  //  counter++;
  //else
  //  counter--;
  //for(int i = 9; i>=0; i--){
  //  sierpinski(0.0,a, b,1.0, c,d, i);
  //  delay(350);
  //}
//     beginShape();
//     fill(255);
//     vertex(0.0,a);
//     vertex(b,1.0);
//     vertex(c,d);
//     endShape();
}


  
  
  
  
  
  
  
//  if(level == 0) {
//   //draw the triangle and get out
//   return;
//  }
//  else{
//    //draw 3 other triangles
//    //coords [(x1, y1),(y1+y2)/2], [ (x1+x2)/2....],[ (x1+x3)/3, .....]
//    sierpinksi(x1,y1, xa,ya, xc,yc, level-1);
//    sierpinski(xa,ya, x2,y2, xb,yb, level-1);
//    sierpinski(xc,yc, xb,yb, x3,y3, level-1);
//  }
//}
//int level = 0;
//void draw() {
//  //coordinates
// sierpinski(0,height-1,width/2,0,width-1,height-1, level); 
//}
