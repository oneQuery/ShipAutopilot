# ShipAutopilot

* USVSimulator_WAMV16.m
  - Plots whole trajectory of the USV.
  - Uses the WAM-V 16 from the Seoul National Univerity.
    
* PathFollowingControl.m
  - Plots sequential position of the vessel.
  - Controls the heading angle with P contorller
  - The variables you can adjust are
  
     ```
     - The limit of the thrust
       line 6 thrustLimit    
     
     - Inital position of the vessel
       line 10 xEarth       
       line 11 yEarth 
       line 12 psi 
      
     - Initial velocity of the vessel
       line 15 uBody
       line 16 vBody
       line 17 r
    
     - Waypoints represented as 
       [[x of first waypoint, y of first waypoint];
       [x of second waypoint, y of second waypoint];
       [x of third waypoint, y of third waypoint];
       ...]]
       line 29 waypoint
    
     - Time interval for simulation 
       line 34 dt
      
     - Reaching criterion to the waypoints
       line 35 reach
      
     - P gain for the P controller
       line 69 pGain ~sdf~
      ```
