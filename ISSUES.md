if you have trouble displaying anything on ubuntu with the simulation, use "xhost +" on ubuntu to solve that 
https://askubuntu.com/questions/115675/xhost-setting-at-boot (maybe look into this for the above issue)


xhost +  # (gives permission to the simulation to host its own x11 server) 
export QT_XCB_GL_INTEGRATION=none   #  (ya know idk what this actually does but it allows me to use x11 as the display type for a qt application. If you don't do this, you get the following error: "Could not initialize GLX")
export XDG_SESSION_TYPE=x11   # (changes the system to host windows using x11 i think? This is for the groundstation)
export QT_QPA_PLATFORM=xcb    # (changes qt to render via x11 for the groundstation)

echo 'xhost +' >> ~/.bashrc
echo 'export QT_XCB_GL_INTEGRATION=none' >> ~/.bashrc
echo 'export XDG_SESSION_TYPE=x11' >> ~/.bashrc
echo 'export QT_QPA_PLATFORM=xcb' >> ~/.bashrc






Maybe also add mount /usr/bin/docker `-v "/usr/bin/docker:/usr/bin/docker"` onto the dev container. 
The main problem is that apparently on mac the place where the docker binary is is actually /usr/local/bin/docker so we would have to make a special case for mac at build time
