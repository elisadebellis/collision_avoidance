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

p_i (x,y): posa ostacolo
t (x,y): posa robot

t - p_i: direzione forza risultante
modulo forza risultante: 1/norm(t_i-p_i)

\sum_i fi + cmd_vel
