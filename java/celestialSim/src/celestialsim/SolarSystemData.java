/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package celestialsim;

import java.io.*;
import java.util.*;

/**
 *
 * @author mycicle
 */
public class SolarSystemData {
    public ArrayList<String> name = new ArrayList<>(); //0
    public ArrayList<Double> mass = new ArrayList<>(); //2
    public ArrayList<Double> diameter = new ArrayList<>(); //3
    public ArrayList<String> orbiting = new ArrayList<>(); //1
    public ArrayList<Double> orad = new ArrayList<>(); //avg of 4, 5
    private ArrayList<Double> perihelion = new ArrayList<>(); //4
    private ArrayList<Double> aphelion = new ArrayList<>(); //5
    private ArrayList<String> output = new ArrayList<>();
    private String path;
    private FileReader fr;
    private BufferedReader br;
    SolarSystemData(String p)throws Exception{
        path = p;
        fr = new FileReader(path);
        br = new BufferedReader(fr);
        
    }
    public void readData()throws Exception{
        String line;
        int counter = 0;
        boolean first = true;
        while ((line = br.readLine()) != null){
            if (first){
                first = false;
            }
            else {
            String[] pInfo = line.split("[\\t\\s][\\t+\\s+]*");
            name.add(new String(pInfo[0]));
            mass.add(new Double(Double.parseDouble(pInfo[2])));
            diameter.add(new Double(Double.parseDouble(pInfo[3])));
            orbiting.add(new String(pInfo[1]));
            orad.add(new Double((Double.parseDouble(pInfo[4]) + Double.parseDouble(pInfo[5]))/2));
            perihelion.add(new Double(Double.parseDouble(pInfo[4])));
            aphelion.add(new Double(Double.parseDouble(pInfo[5])));
            counter ++;
            }
            
        }
        br.close();
    }
    public String toString(){
       
       for(int i = 0; i < mass.size(); i++){
                output.add("Name: " + name.get(i) + '\n' +
                "Mass: " + mass.get(i) + '\n' +
                "Diameter: " + diameter.get(i) + '\n' +
                "Orbiting: " + orbiting.get(i) + '\n' +
                "Orbital Radius: " + orad.get(i) + '\n' + '\n');
        }
       
        return output.toString();
                
                        
    }
    
    public static void main(String[] args)throws Exception{
        SolarSystemData sol = new SolarSystemData("./data/solarsystem.dat");
        sol.readData();
        System.out.println(sol);
        Body[] b = new Body[14];
        Vec3d filler = new Vec3d(1,1,1);
        //public Body(String n, double mass, double radius, double orbital radius, Vec3d position, Vec3d velocity
          //, Vec3d acceleration, double TIMESTEP){
        for (int i = 0; i < 14; i++){
            b[i] = new Body(sol.name.get(i), sol.mass.get(i), (sol.diameter.get(i) / 2), sol.orad.get(i), filler, filler, filler, .2);
        }
        for (int i = 0; i < 14; i++){
            System.out.println(b[i]);
        }
    }
    
}
