package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Welzl's algorithm to find the tighest bounding circle, courtesy of
 * <a href="https://github.com/DustinMorri/Welzl/blob/master/algorithms/Welzl.java">...</a>.
 */
public class Welzl {
    static class Circle {
        double x,y,r;
        Circle(double x,double y,double r){
            this.x=x;
            this.y=y;
            this.r=r;
        }
    }
    static Random r = new Random();
    public static Circle welzl(List<Point> points) {
        ArrayList<Point> pList = new ArrayList<>(points);
        ArrayList<Point> rList = new ArrayList<>();
        return welzl(pList, rList);
    }
    private static Circle welzl(ArrayList<Point> P,ArrayList<Point> R){
        ArrayList<Point> wP = new ArrayList<>(P);
        ArrayList<Point> wR = new ArrayList<>(R);
        if(wP.size()<1||Math.abs(wR.size())>=3){
            if(isConcyclic(wR))
                return makeNewCircle(wR);
            return new Circle(0,0,0);
        }else{
            int rand = r.nextInt(wP.size());
            Point p = wP.remove(rand);
            Circle D = welzl(wP,wR);
            if(isIn(p,D))
                return D;
            wR.add(p);
            return welzl(wP,wR);
        }
    }
    private static boolean isConcyclic(ArrayList<Point> R){
        if(R.size()<2){
            return false;
        }
        else if(R.size()==2){
            return true;
        }
        else if(R.size()==3){
            int different=0;
            double x=R.get(0).x;
            double y=R.get(0).y;
            for(int i=1;i<R.size();i++){
                if(x!=R.get(i).x){
                    different++;
                    break;
                }
            }
            for(int i=1;i<R.size();i++){
                if(y!=R.get(i).y){
                    different++;
                    break;
                }
            }
            return different == 2;
        }
        else {
            ArrayList<Point> T = new ArrayList<>();
            T.add(R.get(0));
            T.add(R.get(1));
            T.add(R.get(2));
            Circle testCircle = makeNewCircle(T);
            for(int i=3;i<R.size();i++){
                if(!(String.format("%.4f",distance(R.get(i).x,R.get(i).y,testCircle.x,testCircle.y))).equals(String.format("%.4f",testCircle.r)))
                    return false;
            }
            return true;
        }
    }
    static Circle makeNewCircle(ArrayList<Point> R){
        if(R.size()==2){
            double x1 = R.get(0).x;//3
            double y1 = R.get(0).y;//5
            double x2 = R.get(1).x;//4
            double y2 = R.get(1).y;//5
            double xd = Math.abs(x2-x1)/2;//0.5
            double yd = Math.abs(y2-y1)/2;//0
            double r = Math.sqrt(s(xd)+s(yd));//0.5
            double x,y;//3.5,5
            if(x1>=x2){
                if(y1>=y2){
                    x = x2+xd;
                    y = y2+yd;
                }else{
                    x = x2+xd;
                    y = y1+yd;
                }
            }else{
                if(y1>=y2){
                    x = x1+xd;
                    y = y2+yd;
                }else{
                    x = x1+xd;
                    y = y1+yd;
                }
            }
            return new Circle(x,y,r);
        }else{
            double x1 = R.get(0).x;
            double y1 = R.get(0).y;
            double x2 = R.get(1).x;
            double y2 = R.get(1).y;
            double x3 = R.get(2).x;
            double y3 = R.get(2).y;
            double d = 2*(x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2));
            double x = (1/d)*((s(x1)+s(y1))*(y2-y3)+(s(x2)+s(y2))*(y3-y1)+(s(x3)+s(y3))*(y1-y2));
            double y = (1/d)*((s(x1)+s(y1))*(x3-x2)+(s(x2)+s(y2))*(x1-x3)+(s(x3)+s(y3))*(x2-x1));
            double r = distance(x,y,x1,y1);
            return new Circle(x,y,r);
        }
    }
    static double s(double n){
        return Math.pow(n,2);
    }
    static boolean isIn(Point p,Circle D){
        double distanceFromCenter = distance(p.x,p.y,D.x,D.y);
        return distanceFromCenter <= D.r;
    }
    static double distance(double x1,double y1,double x2,double y2){
        return Math.sqrt(Math.pow(Math.abs(x1-x2),2)+Math.pow(Math.abs(y1-y2),2));
    }
}