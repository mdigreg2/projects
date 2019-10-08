/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package celestialsim;

/**
 *
 * @author mycicle
 */
public class Vec3d {
    private double x, y, z;
    
    //-------------------------------------------------------------
    public Vec3d(double xi, double yi, double zi){
        x = xi;
        y = yi;
        z = zi;
    }
    public Vec3d(){
        //calls the constructor to send back a 0,0,0 vector
        this(0,0,0);
    }
    //-------------------------------------------------------------
    public double getX(){
        return this.x;
    }
    //-------------------------------------------------------------
     public double getY(){
        return this.y;
    }
    //-------------------------------------------------------------
      public double getZ(){
        return this.z;
    }
    //-------------------------------------------------------------
    public Vec3d add(Vec3d a){
        //to add two vectors  
        //add the x y and z components to get your new vector
        return new Vec3d(this.x+a.x, this.y+a.y, this.z+a.z);
    }
    public static Vec3d add(Vec3d a, Vec3d b){
        return new Vec3d(a.x+b.x, a.y+b.y, a.z+b.z);
    }
    //-------------------------------------------------------------
    public Vec3d sub(Vec3d a){
        return new Vec3d(this.x-a.x, this.y-a.y, this.z-a.z);
    }
    public static Vec3d sub(Vec3d a, Vec3d b){
        return new Vec3d(a.x-b.x, a.y-b.y, a.z-b.z);
    }
    //-------------------------------------------------------------
    public double dot(Vec3d a){
        return (this.x*a.x + this.y*a.y + this.z*a.z);
    }
    public static double dot(Vec3d a, Vec3d b){
        return (a.x*b.x + a.y*b.y + a.z*b.z);
    }
    //-------------------------------------------------------------
    public Vec3d cross(Vec3d a){
        return new Vec3d((this.y*a.z)-(this.z*a.y), 
                         (this.z*a.x)-(this.x*a.z), 
                         (this.x*a.y)-(this.y*a.x));
    }
    public static Vec3d cross(Vec3d a, Vec3d b){
        return new Vec3d((a.y*b.z)-(a.z*b.y), 
                         (a.z*b.x)-(a.x*b.z), 
                         (a.x*b.y)-(a.y*b.x));
    }
    //-------------------------------------------------------------
    public Vec3d toUnitVector(){
        double mag = java.lang.Math.sqrt(x*x + y*y + z*z);
        return divide(this, mag);
    }
    public static Vec3d toUnitVector(Vec3d v){
        double mag = java.lang.Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        return divide(v, mag);
    }
      //-------------------------------------------------------------
        public double magnitude(){
        double mag = java.lang.Math.sqrt(x*x + y*y + z*z);
        return mag;
    }
    public static double magnitude(Vec3d v){
        double mag = java.lang.Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        return mag;
    }
    //-------------------------------------------------------------
        public Vec3d divide(double mag){
        x = x/mag;
        y = y/mag;
        z = z/mag;
        return new Vec3d(x,y,z);
    }
    public static Vec3d divide(Vec3d v, double mag){
        v.x = v.x/mag;
        v.y = v.y/mag;
        v.z = v.z/mag;
        return new Vec3d(v.x,v.y,v.z);
    }
     //-------------------------------------------------------------
     public Vec3d multiply(double mag){
         Vec3d result = new Vec3d(x*mag, y*mag, z*mag);
         return result;
     }
     public static Vec3d multiply(Vec3d v, double mag){
         Vec3d result = new Vec3d(v.x*mag, v.y*mag, v.z*mag);
         return result;
     }
     //-------------------------------------------------------------
    public String toString(){
        return "<"+this.x+","+this.y+","+this.z+">";
    }
    //-------------------------------------------------------------
    public static void main(String[] args){
        Vec3d a = new Vec3d(1,2,3);
        Vec3d b = new Vec3d(1,2,3);
        Vec3d c = new Vec3d(3,7,8);
        
        System.out.println(a);
        System.out.println(b);
        System.out.println(c);
        
        System.out.println(a.add(b));
        System.out.println(add(a,b));
        System.out.println(add(c,b));
        
        System.out.println(a.sub(b));
        System.out.println(sub(a,b));
        System.out.println(sub(a,c));
        
        System.out.println(a.dot(b));
        System.out.println(dot(a,b));
        System.out.println(dot(c,b));
        
        //The cross product of parallel vectors is 0
        System.out.println(a.cross(b));
        System.out.println(cross(a,b));
        System.out.println(cross(a,c));
        
        System.out.println(magnitude(a));
        System.out.println(toUnitVector(a));
    }
}
