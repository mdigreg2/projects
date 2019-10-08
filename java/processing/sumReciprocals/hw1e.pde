/*
AUTHOR: Michael DiGregorio
I pledge my honor that I have abvided by the Stevens Honor System
*/

void setup() {
 float answer = 0;
 for(float i = 1; i <= 100; i ++){
   answer += (1/i);
   print(i+" ");
 }
 print(answer + "\n\n");
 
 float answer2 = 0;
 for(float i = 100; i > 0; i--){
  answer2 += (1/i);
  print(i+" ");
 }
 print(answer2 + "\n\n");
 print("The difference between the two is: " + (answer - answer2));
 exit();
}
