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
public class Body {
    //celestial body needs to have a position, acceleration, velocity
    //mass, name, size, netForce
    
    
    //USE:
    /*
    Body b = new Body(......)
    for (bodies in system):
        b.addForceDueTo(bodies)
    
    b.update()
    
    */
    private Vec3d pos, vel, acc;
    private double mass, rad, orad;
    private String name;
    private Vec3d netForce;
    private double G;
    private static double TMSTP;
    public Body(String n, double m, double r, double or, Vec3d p, Vec3d v, Vec3d a, double TIMESTEP){
        name = n;
        pos = p;
        vel = v;
        acc = a;
        mass = m;
        orad = or;
        rad = r;
        netForce = new Vec3d();
        G = 6.67408e-11;
        TMSTP = TIMESTEP;
    }
    //-------------------------------------------------------------
    //returns the unit vector version of the inputted vector
    //adds a force due to this celestial body
    public void addForceDueTo(Body b){
        //F = Gmm/r^2
        double xDistance2 = java.lang.Math.pow(this.xDist(b), 2);
        double yDistance2 = java.lang.Math.pow(this.yDist(b), 2);
        double zDistance2 = java.lang.Math.pow(this.zDist(b), 2);
        double gravConstant = this.G*b.mass*this.mass;
        double vecX;
        double vecY;
        double vecZ;
        if(xDistance2 != 0){
            vecX = gravConstant / xDistance2;
        }
        else{
            vecX = 0;
        }
        if(yDistance2 != 0){
            vecY = gravConstant / yDistance2;
        }
        else{
            vecY = 0;
        }
        if(zDistance2 != 0){
            vecZ = gravConstant / zDistance2;
        }
        else{
            vecZ = 0;
        }
        
        Vec3d force = new Vec3d(vecX, vecY, vecZ);
        netForce = netForce.add(force);
    }
    //-------------------------------------------------------------
    public double xDist(Body b){
        return this.pos.getX() - b.pos.getX();
    }
    //-------------------------------------------------------------
    public double yDist(Body b){
        return this.pos.getY() - b.pos.getY();
    }
     //-------------------------------------------------------------
    public double zDist(Body b){
        return this.pos.getZ() - b.pos.getZ();
    }
    //-------------------------------------------------------------
    //update velocity, acceleration, position, based on net force
    public void update(){
     // a = F/m
     this.acc = netForce.divide(mass);
     
     //x = xi + vi*t + 1/2 * a * t^2
     Vec3d term1 = vel.multiply(TMSTP);    // vi*t
     Vec3d term2 = acc.multiply((TMSTP*TMSTP)); //at^2
     this.pos = pos.add(term1.add((term2.multiply(.5)))); // pos = xi + vi*t + .5*at^2
     
     //vel = vi + a*t
     this.vel = vel.add(acc.multiply(TMSTP));
     
     //Zero out net force for next time step
     netForce = new Vec3d();
    }
    //-------------------------------------------------------------
        public String toString(){
        return "Body: " + name + '\n' +
               "Net Force: "+ netForce + '\n' +
               "Acceleration: "+ acc + '\n' +
               "Velocity: " + vel + '\n' +
               "Position: " + pos + '\n';
    }
    //-------------------------------------------------------------
    public static void main(String[] args){
        //public Body(String n, double mass, double radius, double orbital radius, Vec3d position, Vec3d velocity
        //, Vec3d acceleration, double TIMESTEP){
        Vec3d posP = new Vec3d(0,0,0);
        Vec3d posE = new Vec3d(0,0,6.378e6);
        Body b = new Body("Person", 100, 1.5, 0,posP, posP, posP, .1);
        Body c = new Body("Earth", 5.972e24, 6.378e6, 1.496e11,posE, posP, posP, .1);
        
        b.addForceDueTo(c);
        c.addForceDueTo(b);
        System.out.println(b);
        System.out.println(c);
        b.update();
        c.update();
        System.out.println(b);
        System.out.println(c);
    }
}
