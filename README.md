# collision_avoidance

Collision Avoidance

un sistema che implementi un sistema anti-collisione basato su laser

Input: 
   - scan, cmd_vel

Output: cmd_vel che non va a sbattere
   - Il sistema prende in input
     - uno scan
     - un comando di velocita'
   - Il sistema produce un comando di velocita', che utilizzando lo scan
     previene la collisione con gli oggetti rilevati nel contorno,
     preferibilmente "deflettendo" la traiettoria del robot verso uno
     spazio libero da collisioni.

----------------------------------------------------------------------------------------------------

Il programma prende in input dqa linea di comando due valori:

///Velocità lineare
float linear_speed;

///Velocità angolare
float angular_speed;

Il laserscan tramite la funzione sensorCallback() rileva la distanza tra il robot e gli ostacoli settando la variabile booleana obstacle_detectected
a true o false 

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorData) {
 
  for (const float &range : sensorData->ranges) {
    if (range < distanceThreshold) {
      setObstacleDetected(true);
      return;
    }
  }
  setObstacleDetected(false);
}


Se l'ostacolo è stato rilevato, il robot si ferma e inizia a ruotare finchè nella sua traiettoria non sarà piu presente l'ostacolo:

   if (checkObstacle()) {
     /// Il robot si ferma e inizia a ruotare
     velocities.linear.x = 0.0;
     velocities.angular.z = angular_speed;
   } 
        
 Altrimenti continua a muoversi con velocità lineare iniziale:
 
   else {
    ///Si muove se l'ostacolo è stato evitato
    velocities.angular.z = 0.0;
    velocities.linear.x = linear_speed;
  }
