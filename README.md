# Gametech

## AIM OF SOFTWARE
To Create a working physical simulator and to generate a maze on a sever that is playable on the client.

## SKILLS DEMONSTRATED
### Physics
* Linear and Angular Newtonian solver.
* Collision Detection with basic shapes
* Basic CPU-modelled soft body.
* Event driven boradphase culling by using octrees that change size depending on the number of members.
### Networks
* Data is generated and executed on servers then displayed on client.
* Keyboard inputs are sent from clients to the server to resize, generate a new map and generate an Avatar.
* Artificial Intellegence:
* Avatar Navagation using the A-Star search algorithm


## Instructions
The Physics and Networks demonstrations can be accessed from diffirent projects

### Physics
#### Running the Project
Visual Studio as well as an OpenGL 3.1 and CUDA enabled GPU are required to generate the executables, to generate them, set GameTech Coursework as the startup project then select any configuration and x64 as the platform, finally click on build or the local windows debugger.

_Note: Release mode will have a higher frame rate than Debug Mode_

#### Controls

* __WASD__: Move Camera

* __up and down arrow keys__: Black Box

* __left and right arrow keys__: Black Box

* __P__: Engine Pause.

* __V__: Toggle V-Sync

* __G__: Toggle detailed performance view

* __Z__: Ball

* __T__: Last Scene

* __Y__: Next Scene

* __Mouse Drag__: Change Pitch and Yaw (Drag boxes when highlighting them)

* __ESC__: Quit

### Networks
#### Running the Project
Visual Studio as well as an OpenGL 3.1 enabled GPU is required to generate the executables, to generate them, right click on the solution and pick properties, select multiple start up projects and pick the Tuts_Network_Client and Tuts_Network Server then select any configuration and x86 as the platform, finally click on build or the local windows debugger.

#### Running Multiple Clients
To run a second client run the executable of that file. This will allow both clients to view the same map and send information with the effects being present in both clients.

#### Client Controls

* __WASD__: Move Camera

* __T__: Last Scene _(only one scene)_

* __Y__: Next Scene _(only one scene)_

* __R__: Reload Scene

* __1__: Regenerate Map

* __2__: Draw Line between start point and goal

* __3__: Generate Avatar to head from start point to goal

* __4__: Decrease Size of maze

* __5__: Increase Size of maze

* __Mouse Drag__: Change Pitch and Yaw (Drag boxes when highlighting them)

* __ESC__: Quit

## Citations
* OpenGL code based on [Newcastle University Game Technologies Framework](

## Citations
* OpenGL code based on [Newcastle University Graphics Framework](https://research.ncl.ac.uk/game/mastersdegree/graphicsforgames/)
