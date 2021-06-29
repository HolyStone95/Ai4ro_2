    /*
     <one line to give the program's name and a brief idea of what it does.>
     Copyright (C) 2015  <copyright holder> <email>
     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     */


#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "armadillo"
#include <initializer_list>

using namespace std;
using namespace arma;



    //map <string, vector<double> > region_mapping;

extern "C" ExternalSolver* create_object(){
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver){
  delete externalSolver;
}

VisitSolver::VisitSolver(){

}

VisitSolver::~VisitSolver(){

}

void VisitSolver::loadSolver(string *parameters, int n){
  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[]={"dummy"};
  char const *y[]={"act-cost","triggered"};
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  string waypoint_file = "/root/ai4ro2/visits_domain/waypoint.txt";
  parseWaypoint(waypoint_file);

  string landmark_file = "/root/ai4ro2/visits_domain/landmark.txt";
  parseLandmark(landmark_file);


        //startEKF();
}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic){

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  double dummy = 0;
  double act_cost = 0;
  double dis = 0;
  double unc = 0;

  map<string, double> trigger;

  for(;iSIt!=isEnd;++iSIt){

    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0,1);
    function.erase(function.length()-1,function.length());
    int n=function.find(" ");

    if(n!=-1){
      string arg=function;
      string tmp = function.substr(n+1,5);

      function.erase(n,function.length()-1);
      arg.erase(0,n+1);
      if(function=="triggered"){
        trigger[arg] = value>0?1:0;
        if (value>0){

      string from = tmp.substr(0,2);   // from and to are regions, need to extract wps (poses)
      string to = tmp.substr(3,2);


      localize(from, to);
      localize2(to);
      //cout << "unc"<<unc << endl;
       }
      }
    }
    else{
      if(function=="dummy"){
        dummy = value;
      }
      else if(function=="act-cost"){
        act_cost = value;
      } //else if(function=="dummy1"){
        //duy = value;              
        ////cout << parameter << " " << value << endl;
        //}
    }
  }

  double results = calculateExtern(dummy, act_cost);
  toReturn["(dummy)"] = results;


return toReturn;
}


            list<string> VisitSolver::getParameters(){

              return affected;
            }

            list<string> VisitSolver::getDependencies(){

              return dependencies;
            }


            void VisitSolver::parseParameters(string parameters){

              int curr, next;
              string line;
              ifstream parametersFile(parameters.c_str());
              if (parametersFile.is_open()){
                while (getline(parametersFile,line)){
                 curr=line.find(" ");
                 string region_name = line.substr(0,curr).c_str();
                 curr=curr+1;
                 while(true ){
                  next=line.find(" ",curr);
                  region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
                  if (next ==-1)
                   break;
                 curr=next+1;

               }                
             }

           }

         }

double VisitSolver::calculateExtern(double external, double total_cost){
  //float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
  double cost = distance + unc;//random1;
  cout << "cost :  " << cost << "," << "distance : " << distance << "," << "trace : " << unc << endl;


return cost;
}

     void VisitSolver::parseWaypoint(string waypoint_file){

       int curr, next;
       string line;
       double pose1, pose2, pose3;
       ifstream parametersFile(waypoint_file);
       if (parametersFile.is_open()){
        while (getline(parametersFile,line)){
         curr=line.find("[");
         string waypoint_name = line.substr(0,curr).c_str();

         curr=curr+1;
         next=line.find(",",curr);

         pose1 = (double)atof(line.substr(curr,next-curr).c_str());
         curr=next+1; next=line.find(",",curr);

         pose2 = (double)atof(line.substr(curr,next-curr).c_str());
         curr=next+1; next=line.find("]",curr);

         pose3 = (double)atof(line.substr(curr,next-curr).c_str());

         waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};
       }
     }

   }

   void VisitSolver::parseLandmark(string landmark_file){

     int curr, next;
     string line;
     double pose1, pose2, pose3;
     ifstream parametersFile(landmark_file);
     if (parametersFile.is_open()){
      while (getline(parametersFile,line)){
       curr=line.find("[");
       string landmark_name = line.substr(0,curr).c_str();
       
       curr=curr+1;
       next=line.find(",",curr);

       pose1 = (double)atof(line.substr(curr,next-curr).c_str());
       curr=next+1; next=line.find(",",curr);

       pose2 = (double)atof(line.substr(curr,next-curr).c_str());
       curr=next+1; next=line.find("]",curr);

       pose3 = (double)atof(line.substr(curr,next-curr).c_str());

       landmark[landmark_name] = vector<double> {pose1, pose2, pose3};
     }
   }
   
 }


  void VisitSolver::localize( string from, string to){
    
    string wp[5] = {"wp0","wp1","wp2","wp3","wp4"};
    map <string, string> reg_wp;
    reg_wp["r0"] = wp[0];
    reg_wp["r1"] = wp[1];
    reg_wp["r2"] = wp[2];
    reg_wp["r3"] = wp[3];
    reg_wp["r4"] = wp[4];

    double x1 = waypoint[reg_wp[from]].at(0);
    double y1 = waypoint[reg_wp[from]].at(1);
    
    double x2 = waypoint[reg_wp[to]].at(0);
    double y2 = waypoint[reg_wp[to]].at(1);
    
    distance = sqrt(pow((x2-x1),2) + pow((y2-y1),2));
  }

  void VisitSolver::localize2(string to){

    string wp[5] = {"wp0","wp1","wp2","wp3","wp4"};
    map <string, string> reg_wp;
    reg_wp["r0"] = wp[0];
    reg_wp["r1"] = wp[1];
    reg_wp["r2"] = wp[2];
    reg_wp["r3"] = wp[3];
    reg_wp["r4"] = wp[4];
    double x = waypoint[reg_wp[to]].at(0);
    double y = waypoint[reg_wp[to]].at(1);
    double theta = waypoint[reg_wp[to]].at(2);
    string land[] = {"l1","l2","l3","l4"};

    double xl = landmark[land[0]].at(0);
    double yl = landmark[land[0]].at(1);
    double x2 = landmark[land[1]].at(0);
    double y2 = landmark[land[1]].at(1);
    double x3 = landmark[land[2]].at(0);
    double y3 = landmark[land[2]].at(1);
    double x4 = landmark[land[3]].at(0);
    double y4 = landmark[land[3]].at(1);

    std::vector<std::vector<double>> Pinit(3, std::vector<double> (3));
    std::vector<std::vector<double>> I(3, std::vector<double> (3));
    std::vector<std::vector<double>> C(8, std::vector<double> (3));
    std::vector<std::vector<double>> K(3, std::vector<double> (8));
    std::vector<std::vector<double>> C_P(8, std::vector<double> (3));
    std::vector<std::vector<double>> C_P_C(8, std::vector<double> (8));
    std::vector<std::vector<double>> Ct(3, std::vector<double> (8));
    std::vector<std::vector<double>> Q_gamma(8, std::vector<double> (8));

    Pinit = {
        {0.02, 0 , 0},
        {0 , 0.02 , 0},
        {0, 0, 0.02}
    };
    
    I = {
        {1, 0 , 0},
        {0 , 1 , 0},
        {0, 0, 1}
    };
    
    Q_gamma = 
     {{0.2, 0, 0, 0, 0, 0, 0, 0},
      {0, 0.2, 0, 0, 0, 0, 0, 0},
      {0, 0, 0.2, 0, 0, 0, 0, 0},
      {0, 0, 0, 0.2, 0, 0, 0, 0},
      {0, 0, 0, 0, 0.2, 0, 0, 0},
      {0, 0, 0, 0, 0, 0.2, 0, 0},
      {0, 0, 0, 0, 0, 0, 0.2, 0},
      {0, 0, 0, 0, 0, 0, 0, 0.2},
    };

    C = {
        {-cos(theta), -sin(theta) , (-sin(theta) * (xl - x)) + ( cos(theta)*(yl - y)) },
        {sin(theta) , -cos(theta) , (-sin(theta) * (yl - y)) + ( -cos(theta)*(xl - x)) },
        {-cos(theta), -sin(theta) , (-sin(theta) * (x2 - x)) + ( cos(theta)*(y2 - y)) },
        {sin(theta) , -cos(theta) , (-sin(theta) * (y2 - y)) + ( -cos(theta)*(x2 - x)) },
        {-cos(theta), -sin(theta) , (-sin(theta) * (x3 - x)) + ( cos(theta)*(y3 - y)) },
        {sin(theta) , -cos(theta) , (-sin(theta) * (y3 - y)) + ( -cos(theta)*(x3 - x)) },
        {-cos(theta), -sin(theta) , (-sin(theta) * (x4 - x)) + ( cos(theta)*(y4 - y)) },
        {sin(theta) , -cos(theta) , (-sin(theta) * (y4 - y)) + ( -cos(theta)*(x4 - x)) },
    };
    C_P=Multipl(C , Pinit);
    Ct = getTranspose(C);
    C_P_C=Multipl(C_P , Ct);
    K = Multipl(Multipl(Pinit , getTranspose(C)), getInverse(Add(C_P_C, Q_gamma)));
    Pinit = Multipl(Sub( I , Multipl(K, C)), Pinit);
    unc = Pinit[0][0] + Pinit[1][1] + Pinit[2][2];
    unc *= 20;
  }

///------------------------------------------------------------------------------------------------------------------

double VisitSolver::getDeterminant(const std::vector<std::vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 
    int dimension = vect.size();
    if(dimension == 0) {
        return 1;
    }

    if(dimension == 1) {
        return vect[0][0];
    }

    //Formula for 2x2-matrix
    if(dimension == 2) {
        return vect[0][0] * vect[1][1] - vect[0][1] * vect[1][0];
    }

    double result = 0;
    int sign = 1;
    for(int i = 0; i < dimension; i++) {
        //Submatrix
        std::vector<std::vector<double>> subVect(dimension - 1, std::vector<double> (dimension - 1));
        for(int m = 1; m < dimension; m++) {
            int z = 0;
            for(int n = 0; n < dimension; n++) {
                if(n != i) {
                    subVect[m-1][z] = vect[m][n];
                    z++;
                }
            }
        }
        //recursive call
        result = result + sign * vect[0][i] * getDeterminant(subVect);
        sign = -sign;
    }
    return result;
}

std::vector<std::vector<double>> VisitSolver::getTranspose(const std::vector<std::vector<double>> matrix1) {

    //Transpose-matrix: height = width(matrix), width = height(matrix)
    std::vector<std::vector<double>> solution(matrix1[0].size(), std::vector<double> (matrix1.size()));

    //Filling solution-matrix
    for(size_t i = 0; i < matrix1.size(); i++) {
        for(size_t j = 0; j < matrix1[0].size(); j++) {
            solution[j][i] = matrix1[i][j];
        }
    }
    return solution;
}

std::vector<std::vector<double>> VisitSolver::getCofactor(const std::vector<std::vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 

    std::vector<std::vector<double>> solution(vect.size(), std::vector<double> (vect.size()));
    std::vector<std::vector<double>> subVect(vect.size() - 1, std::vector<double> (vect.size() - 1));

    for(std::size_t i = 0; i < vect.size(); i++) {
        for(std::size_t j = 0; j < vect[0].size(); j++) {

            int p = 0;
            for(size_t x = 0; x < vect.size(); x++) {
                if(x == i) {
                    continue;
                }
                int q = 0;

                for(size_t y = 0; y < vect.size(); y++) {
                    if(y == j) {
                        continue;
                    }

                    subVect[p][q] = vect[x][y];
                    q++;
                }
                p++;
            }
            solution[i][j] = pow(-1, i + j) * getDeterminant(subVect);
        }
    }
    return solution;
}

std::vector<std::vector<double>> VisitSolver::getInverse(const std::vector<std::vector<double>> vect) {
    if(getDeterminant(vect) == 0) {
        throw std::runtime_error("Determinant is 0");
    } 

    double d = 1.0/getDeterminant(vect);
    std::vector<std::vector<double>> solution(vect.size(), std::vector<double> (vect.size()));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] = vect[i][j]; 
        }
    }

    solution = getTranspose(getCofactor(solution));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] *= d;
        }
    }
    return solution;
}

void VisitSolver::printMatrix(const std::vector<std::vector<double>> vect) {
    for(std::size_t i = 0; i < vect.size(); i++) {
        for(std::size_t j = 0; j < vect[0].size(); j++) {
            std::cout << std::setw(8) << vect[i][j] << " ";
        }
        std::cout << "\n";
    }
}

std::vector<std::vector<double>> VisitSolver::Multipl(const std::vector<std::vector<double>> first , const std::vector<std::vector<double>> second) {

  std::vector<std::vector<double>> mul(first.size(), std::vector<double> (second[0].size()));
  for(unsigned int i=0;i<first.size();i++) {  
    for(unsigned int j=0;j<second[0].size();j++) {
      mul[i][j]=0;    
      for(unsigned int k=0;k<first[0].size();k++){ 
        mul[i][j]+=first[i][k]*second[k][j];    
      }    
    }    
  }
return mul;
}

std::vector<std::vector<double>> VisitSolver::Sub(const std::vector<std::vector<double>> first , const std::vector<std::vector<double>> second) {

  std::vector<std::vector<double>> sub(first.size(), std::vector<double> (second[0].size()));
  for(unsigned int i=0;i<first.size();i++) {    
    for(unsigned int j=0;j<second[0].size();j++) {    
      sub[i][j] = first[i][j] - second[i][j];  
    }    
  }
return sub;
}

std::vector<std::vector<double>> VisitSolver::Add(const std::vector<std::vector<double>> first , const std::vector<std::vector<double>> second) {

  std::vector<std::vector<double>> sub(first.size(), std::vector<double> (second[0].size()));
  for(unsigned int i=0;i<first.size();i++) {    
    for(unsigned int j=0;j<second[0].size();j++) {    
      sub[i][j] = first[i][j] + second[i][j];  
    }    
  }
return sub;
}




