/*
AUTHOR: Michael DiGregorio
I pledge my honor that I have abided by the Stevens Honor System
*/


void setup() {
 //print the odd numbers from 1 - 99 followed by a newline
 
 for(int i = 1; i<=99; i+=2){
  print(i + " "); 
 }
  println("\n");
  
  for(int i = 0; i <= 15; i++){
    print(pow(2,i) + " ");
  }
  println("\n");
  exit();
}
