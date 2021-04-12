#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/DebugValue.h>
#include <std_msgs/String.h>
#include <math.h>  
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <cstdlib>

using namespace std;

// Estructura para guardar informacion de la vaca encontrada
struct DebugVector{
	string cowID;
	float x,y,z;
};


string datos = "Vaca134,55.135,2.983,3.352";

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

std_msgs::String current_cow;
void cow_cb(const std_msgs::String::ConstPtr& msg){
    current_cow = *msg;
}

// Funcion divisora de strings usando un delimitador
DebugVector tokenize(string s, string del = " ")
{
    DebugVector debVec;
    int start = 0;
    int counter = 0;
    int end = s.find(del);
    while (end != -1) {
        cout << s.substr(start, end - start) << endl;
	if (counter == 0)
        	debVec.cowID = s.substr(start, end - start); // Obtiene ID de la vaca
	else if (counter == 1)
		debVec.x = stof(s.substr(start, end - start)); // Obtiene latitud de la vaca y convierte a float
	else if (counter == 2)
		debVec.y = stof(s.substr(start, end - start)); // Obtiene latitud de la vaca y convierte a float
	counter++;
        start = end + del.size();
        end = s.find(del, start);
    }
    cout << s.substr(start, end - start) << endl;
    debVec.z = stof(s.substr(start, end - start)); // Obtiene altitud de la vaca y la convierte a float
    return debVec;
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "debug_vector");
    ros::NodeHandle nh;

    // Suscriptor del topico mavros/state 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // Suscriptor del topico cows_found (rosserial arduino)
    ros::Subscriber rosserial_sub = nh.subscribe<std_msgs::String>
            ("cows_found", 10, cow_cb);
    // Publica en el topico DEBUG_VECT        
    ros::Publisher debug_pub = nh.advertise<mavros_msgs::DebugValue>
            ("mavros/debug_value/send", 20);

    ros::Rate rate(20.0);

    // Espera a tener conexión con la Pixhawk
    // UNCOMMENT THIS!!!!!
//    while(ros::ok() && !current_state.connected){
//        ros::spinOnce();
//        rate.sleep();
//    }

    // Para almacenar informacion de la vaca encontrada 
    mavros_msgs::DebugValue debb;
    debb.header.stamp = ros::Time::now();
    debb.header.frame_id = ""; //ID de dron
    debb.name = "";
    debb.type = 1;  // mensaje del tipo DEBUG_VECT
    debb.index = -1;
    debb.data = {0.0, 0.0, 0.0} ;

    while(ros::ok()){

	    DebugVector debVec;
	    if (current_cow.data.substr(0,3) == "cow") // Si se ha encontrado una vaca
		debVec = tokenize(current_cow.data,","); // "," es el delimitador
	    else
	    	debVec = tokenize(datos,",");

	    //Solving precision issues by sending a multiplied value (the amount of zeros depends)
//	    float zeros = 100000; 
//	    int x_int = (int)(debVec.x*zeros);
//	    int y_int = (int)(debVec.y*zeros);
//	    int z_int = (int)(debVec.z*zeros);

            debb.header.stamp = ros::Time::now();
	    debb.name = current_cow.data;
//	    debb.name = debVec.cowID;
            debb.data[0] = debVec.x;
            debb.data[1] = debVec.y;
            debb.data[2] = debVec.z;
//            debb.data[0] = (float)x_int;
//            debb.data[1] = (float)y_int;
//            debb.data[2] = (float)z_int;
     
//            debb.data[0] = stof(debVec.x);
//            debb.data[1] = stof(debVec.y);
//            debb.data[2] = stof(debVec.z);
     
        
        debug_pub.publish(debb); // Publica el DEBUG_VECTOR
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
