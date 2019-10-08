/*
AUTHOR: Michael DiGregorio
I pledge my honor that I have abided by the Stevens Honor System
*/


//skip 75 pixels over and then connect lines

void setup() {
 size(800,800); 
  
}

void draw() {
  int x = 750;
 for(int i = 0; i <= x; i += 75){
  for(int j = 0; j <= x; j += 75){
   line(i,0, 0,j); 
  }
 }
}
