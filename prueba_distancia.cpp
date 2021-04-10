// Recibe una PointCloud2 en el marco del dron, la convierte en PointCloud (x,y,z), obtiene el histograma primario, histograma binario e histograma binario alargado

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>
#include <math.h>  
#include <stdio.h>

using namespace std; 
const float  PI = 3.14159265358979f;

sensor_msgs::PointCloud point_cloud;


void pcl_cam_call(const sensor_msgs::PointCloud2ConstPtr& msg){

  sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud);
  //ROS_INFO("Points: %d",point_cloud.points.size());

}

int main(int argc, char** argv){
  ros::init(argc, argv, "map_nav");

  ros::NodeHandle node;

  ROS_INFO("Map_nav ready...");

  //PointCloud2 in DRONE FRAME:
  ros::Subscriber pcl2_cam_sub = node.subscribe("transformed_pcl2", 1, pcl_cam_call);

  ros::Publisher pcl_pub = 
    node.advertise<sensor_msgs::PointCloud>("pcl_base", 1);

  ros::Rate rate(20.0);

  while (node.ok()){
    int sum = 0;
    int res = 3; //resolucion en grados realsense
    float ang_az, ang_el; // angulos de azimuth y elevación
//    float fov_x = 87, fov_y= 57; //FOV en grados relasense
    float fov_x = 75, fov_y= 54; //FOV en grados relasense
    int index_x, index_y;
    const int size_x = fov_x/res, size_y = fov_y/res; 
    float max_range = 5.0;

    int histo_prim[size_x][size_y]; //cuenta de puntos
    float histo_dis[size_x][size_y]; //promedio de distancias o distancia mínima

    for (int i = 0; i < size_x; i++) 
    { 
       for (int j = 0; j < size_y; j++) 
       { 
          histo_prim[i][j] = 0;
          histo_dis[i][j] = max_range; //max. rango
       } 
    }

    
    cout << "Total points: " << point_cloud.points.size() << endl;  


    // CAPA DE DISTANCIAS TEORICA
    float histo_dis_teo[size_x][size_y];
    float dist_pared = 1.15;

    for (int i = 0; i < size_x; i++) 
    { 
       for (int j = 0; j < size_y; j++) 
       {
	  float az_teo = res * (i + 0.5) - fov_x/2;
	  float el_teo = res * (j + 0.5) - fov_y/2;
	  
	  histo_dis_teo[i][j] = dist_pared / cos(PI/180.0 * az_teo);
	  histo_dis_teo[i][j] /= cos(PI/180.0 * el_teo);
       } 
    }

    // IMPRIMIR CAPA DE DISTANCIAS TEORICA
    cout <<"CAPA DE DISTANCIAS TEORICA:" << endl; 
    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
          cout << histo_dis_teo[i][j] << " "; 
       } 
       // Newline for new row 
       cout << endl; 
    }
    cout << endl; 
    cout << endl; 
    cout << endl;



    // HISTOGRAMA PRIMARIO Y CAPA DE DISTANCIAS

    for (int i = 0; i < point_cloud.points.size(); i++){
      
      // Getting distance to the point
      float dis;// = point_cloud.points[i].x;
      dis = pow(point_cloud.points[i].x,2) + pow(point_cloud.points[i].y,2) + pow(point_cloud.points[i].z,2);
      dis = sqrtf(dis);
      //sum = sum + dis;
      if (dis >= 0.0 && dis <= max_range){
      // Getting azimuth and elevation angles
        ang_az = 180.0/PI * atan2(point_cloud.points[i].y, point_cloud.points[i].x) + fov_x/2;
        ang_el = point_cloud.points[i].z / (pow(point_cloud.points[i].x,2) + pow(point_cloud.points[i].y,2));
        ang_el = 180.0/PI * atan(ang_el) + fov_y/2;
        //ROS_INFO("Azimuth: %f,   Elevacion: %f",ang_az,ang_el);
        
        if ( ang_az >= 0.0 && ang_az <= fov_x && ang_el >= 0.0 && ang_el <= fov_y){
          sum ++;
          // Getting index for the point
          index_x = (int)ang_az/res;
          index_y = (int)ang_el/res;

          histo_prim[index_x][index_y]++; //Adding point to the historgam

          if (dis < histo_dis[index_x][index_y]){
            histo_dis[index_x][index_y] = dis; //Saving the minimum distance
          }
        }
       }
    }

    cout << "Counted points: " << sum << endl; 
    
    //Printing capa de distancias del histograma primario
    cout <<"CAPA DE DISTANCIAS OBTENIDA:" << endl; 
    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
        if (histo_prim[i][j] == 0)
          {
            histo_dis[i][j] = 0.0;
          }
          cout << fixed << setprecision(2) << histo_dis[i][j] << " "; 
       } 
       // Newline for new row 
       cout << endl; 
    }
    cout << endl; 


    // IMPRIMIR DIFERENCIA ENTRE TEORICO Y REAL
    cout <<"DIFERENCIA ABSOLUTA ENTRE TEORICO Y OBTENIDO:" << endl; 
    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
	  float dif = abs(histo_dis_teo[i][j] - histo_dis[i][j]);
          cout << dif << " "; 
       } 
       // Newline for new row 
       cout << endl; 
    }
    cout << endl; 
    cout << endl; 
    cout << endl;


    // HISTOGRAMA BINARIO

    bool histo_bin[size_x][size_y]; // Histograma binario
    float thresh = 2.1; // Limite de distancia para considerar como ocupado

    for (int i = 0; i < size_x; i++) 
    { 
       for (int j = 0; j < size_y; j++) 
       {
        if (histo_dis[i][j] != 0.0 && histo_dis[i][j] <= thresh && histo_prim[i][j] > 100) // Se considera la distancia y cuantos puntos hay en cada celda
        {
          histo_bin[i][j] = true;
        }
        else
        {
          histo_bin[i][j] = false;
        }
       }
    }
    
    // IMPRIMIR HISTOGRAMA BINARIO
    cout << endl; 
    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
          cout << histo_bin[i][j] << " "; 
       } 
       // Newline for new row 
       cout << endl; 
    }
    cout << endl; 
    cout << endl; 
    cout << endl;


    // ALARGAMIENTO HISTOGRAMA BINARIO
    bool histo_bin_a[size_x][size_y];
    float r_alarg = 0.25; // Radio de alargamiento en metros

    for (int i = 0; i < size_x; i++) 
    { 
      for (int j = 0; j < size_y; j++) 
      {
        histo_bin_a[i][j] = 0;
      }
    }


    for (int i = 0; i < size_x; i++) 
    { 
      for (int j = 0; j < size_y; j++) 
      {
        if (histo_bin[i][j] == 1)
        {
          histo_bin_a[i][j] = 1;

          float lambda = 180.0 / PI * asin(r_alarg / histo_dis[i][j]); 
          int lambda_int = (int)lambda/res; // Cuantas celdas se alarga

          for (int ii = i-lambda_int; ii <= i+lambda_int; ii++){
            for (int jj = j-lambda_int; jj <= j+lambda_int; jj++){
              if (ii >= 0 && jj >= 0 && ii < size_x && jj < size_y){
                histo_bin_a[ii][jj] = 1;
              }
            }
          }
        }
      }
    }

    // IMPRIMIR HISTOGRAMA BINARIO ALARGADO
    cout << endl; 
    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
          cout << histo_bin_a[i][j] << " "; 
       } 
       // Newline for new row 
       cout << endl; 
    }
    cout << endl; 
    cout << endl; 
    cout << endl;

    pcl_pub.publish(point_cloud);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
