# ShipAutopilot

* USVSimulator_WAMV16.m
  - Plots whole trajectory of the USV.
  - Uses the WAM-V 16 from the Seoul National Univerity.
    
* PathFollowingControl.m
  - Plots sequential position of the vessel.
  - Controls the heading angle with P contorller
  - The variables you can adjust are
  
    >  ```
    > % The limit of the thrust
    > line 6 thrustLimit    
    > 
    > % Inital position of the vessel
    > line 10 xEarth       
    > line 11 yEarth 
    > line 12 psi 
    >  
    > % Initial velocity of the vessel
    > line 15 uBody
    > line 16 vBody
    > line 17 r
    >
    > % Waypoints represented as [[x1, y1]; [x2, y2]; [x3, y3]; ...]
    >  line 29 waypoint
    >  
    >  line 45 dt
    >  
    >  line 35 reach
    >  
    >  line 57 pGain ~sdf~
    >  ```
