Making emergency stop stuff and updating the sailboat state machine stuff

MADE IT SO THAT THE AUTOPILOT DOESN'T CRASH WHEN TRYING TO SWITCH TO WAYPOINT MISSION MODE WHEN THERE ARE NO WAYPOINTS SET. ALSO FIXED THIS FOR THE MOTORBOAT AUTOPILOT.
Made it so that the motorboat autopilot returns None for the rudder angle when we reach the final waypoint

Made some misc modifications to the sailboat state machine

Changed how the desired_heading is set in the sailboat autopilot

TODO:

Make cv parameters editable from the groundstation
