/*
AUTHOR: Michael DiGregorio
I pledge my honor that I have abided by the Stevens Honor System
*/


void setup(){
 size(400, 400); 
}
final int N = 8;
void draw(){
  for(int i = 0; i < width; i+=(width/N)){
    line(i,0,i, height);
  }
  for(int i = 0; i < height; i+=(height/N)){
   line(0,i, width, i); 
  }

}
