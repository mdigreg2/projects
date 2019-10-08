/*
AUTHOR: Michael DiGregorio
I pledge my honor that I have abided by the Stevens Honor System
*/

//----------------------------------------------------------------------------------------------------------------
//integer parameter and returns a factorial iteratively
double fact(int n){
  if(n == 0)
    return 1;
  double num = 1;
  for(int i = 1; i <=n; i++){
    num *= i;
  }
 return  num;
}
//----------------------------------------------------------------------------------------------------------------
//takes an integer parameter and returns a factorial recursively
double fact2(int n){
  double num = n;
  if(n < 0){
   return 0; 
  }
  if(n == 1 || n == 0){
   return 1; 
  }
  return (num*fact2(n-1));
}
//----------------------------------------------------------------------------------------------------------------
//return the nth term of the fibonacci sequence iteratively
double fibo(int n){
  if(n == 0)
    return 0;
  if(n == 1)
    return 1;
    
  double prev2 = 0;
  double prev1 = 1;
  double answer = 0;
  for(int i = 1; i < n; i++){
   answer = prev1 + prev2;
   prev2 = prev1;
   prev1 = answer;
  }
 return answer;
}
//----------------------------------------------------------------------------------------------------------------
//return the nth term of the fibonacci sequence recursively (;ets try tail recursion)
double tail_fib(int a, int b, int n){
 if(n == 0)
   return a;
 if(n == 1)
   return b;
 if(n == 2)
   return a+b;
 else
   return tail_fib(b, a+b, n-1); 
}
double fibo2(int n){
 return tail_fib(0, 1, n); 
}
//----------------------------------------------------------------------------------------------------------------

void setup() {
  int n = 30;
  int fib = 8;
  
  
  for(int i = 0; i <=30; i += 3){
    print(fact(i) + " ");
  }
  print('\n');
  
  for(int i = 0; i <=30; i += 3){
    print(fact2(i) + " ");
  }
  print('\n');
  
  for(int i = 1; i<=20; i++){
    print(fibo(i) + " ");
  }
  print('\n');
  
  for(int i = 1; i<=20; i++){
    print(fibo2(i) + " ");
  }
  print('\n');
  exit();
}
