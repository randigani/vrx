**Glare:**  
In the world .sdf file:  
Find the directional light block for the sun:  
   \<light type\="directional" name\="sun"\>  
     \<cast\_shadows\>true\</cast\_shadows\>  
     \<pose\>5 20 10 0 0 0\</pose\>  
     \<diffuse\>1 0.95 0.9 1\</diffuse\>  
     \<specular\>1 1 1 1\</specular\>  
     *\<\!-- Increase ^ the first 3 numbers (up to 1\) for stronger glint \--\>*  
     \<attenuation\>  
       \<range\>1000\</range\>  
       \<constant\>1.0\</constant\>  
       \<linear\>0.00\</linear\>  
       \<quadratic\>0.000\</quadratic\>  
     \</attenuation\>  
     \<direction\>\-0.4 \-0.9 \-0.2\</direction\>  
     *\<\!-- Set the vector ^ so the sun is at a low angle \--\>*  
   \</light\>

These set the sun at an angle and optionally increase glint from the reflecting sun. The first three numbers in \<specular\> are RGB values, the fourth value has no effect (transparency).

In the \<scene\> block:  
   \<scene\>  
      \<sky\>\</sky\>  
      \<grid\>false\</grid\>  
      \<ambient\>0.12 0.12 0.12\</ambient\>  
      *\<\!-- Decrease ^ these ^  for more glare contrast effect  \--\>*  
      \<background\>0.6 0.6 0.6\</background\>  
   \</scene\>

This change makes the shadows more intense, creating the dark glare effect. Like before, the three values are RGB values.

**Wind:**  
In the world .sdf file:  
   \<plugin  
     filename\="libUSVWind.so"  
     name\="vrx::USVWind"\>  
     \<wind\_obj\>  
       \<name\>wamv\</name\>  
       \<link\_name\>wamv/base\_link\</link\_name\>  
       \<coeff\_vector\>.5 .5 .33\</coeff\_vector\>  
     \</wind\_obj\>  
     *\<\!-- Wind \--\>*  
     \<wind\_direction\>240\</wind\_direction\>  
     *\<\!-- in degrees \--\>*  
     \<wind\_mean\_velocity\>5.0\</wind\_mean\_velocity\>  
     *\<\!-- Change the value ^ for wind velocity \--\>*  
     \<var\_wind\_gain\_constants\>0\</var\_wind\_gain\_constants\>  
     \<var\_wind\_time\_constants\>2\</var\_wind\_time\_constants\>  
     \<random\_seed\>10\</random\_seed\>  
     *\<\!-- set to zero/empty to randomize \--\>*  
     \<update\_rate\>10\</update\_rate\>  
     \<topic\_wind\_speed\>/vrx/debug/wind/speed\</topic\_wind\_speed\>  
     \<topic\_wind\_direction\>/vrx/debug/wind/direction\</topic\_wind\_direction\>  
   \</plugin\>

[VRX exposes](https://github.com/osrf/vrx/wiki/wind_params_tutorial) a bunch of parameters for wind simulation. Change wind direction, velocity, modulation, etc. 

**Waves:**  
In the world .sdf file:  
   \<plugin filename\="libPublisherPlugin.so" name\="vrx::PublisherPlugin"\>  
     \<message type\="gz.msgs.Param" topic\="/vrx/wavefield/parameters"  
              every\="2.0"\>  
       params {  
         key: "direction"  
         value {  
           type: DOUBLE  
           double\_value: 0.0  
         }  
       }  
       params {  
         key: "gain"  
         value {  
           type: DOUBLE  
           double\_value: 3.0  
         }  
       }  
       params {  
         key: "period"  
         value {  
           type: DOUBLE  
           double\_value: 5  
         }  
       }  
       params {  
         key: "steepness"  
         value {  
           type: DOUBLE  
           double\_value: 0  
         }  
       }  
     \</message\>  
   \</plugin\>

Native [VRX plugin](https://github.com/osrf/vrx/wiki/wave_params_tutorial), Gain controls the height of the waves, period controls the wave period. Direction didn’t really seem to do anything (Could just be me), steepness apparently changes the waves shape.

**Ocean Current:**  
Add this to wamv\_gazebo.urdf.xacro, under the other \<gazebo\> blocks:  
 \<gazebo\>  
   \<plugin filename\="gz-sim-hydrodynamics-system"  
           name\="gz::sim::systems::Hydrodynamics"\>  
     \<link\_name\>wamv/base\_link\</link\_name\>  
     \<default\_current\>0.00001 0.0 0.0\</default\_current\>  
     *\<\!-- /ocean\_current topic only works when this is non-zero...?  \--\>*

     *\<\!-- Linear damping \--\>*  
     \<xU\>\-5.0\</xU\>  
     \<yV\>\-5.0\</yV\>  
     \<zW\>\-5.0\</zW\>  
     \<kP\>\-1.0\</kP\>  
     \<mQ\>\-1.0\</mQ\>  
     \<nR\>\-1.0\</nR\>

     *\<\!-- Quadratic damping \--\>*  
     \<xUabsU\>\-100.0\</xUabsU\>  
     \<yVabsV\>\-100.0\</yVabsV\>  
   \</plugin\>  
 \</gazebo\>

This plugin uses [Gazebo’s realistic hydrodynamic physics](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1systems_1_1Hydrodynamics.html), and takes damping values. You can change the values in \<default\_current\>, or with the simulation running enter in the terminal:  
`gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 2.0, y: -0.0, z: 0.0'`  
Change the values in the quotes for desired current velocity. Don’t set \<default\_current\> to   
0 0 0 , since this causes the above command to lose its effect for some reason. 

Higher damping correlates to how strongly the current wants to take the USV with it.   
Default values (zero for everything) eliminates any hydrodynamic effect on the USV.

One pitfall: Since this isn’t a VRX bespoke plugin, the hydrodynamic force is applied to the whole USV mesh, not just the submersed portion. This shouldn’t be an issue at reasonable ocean current speeds, and can probably be compensated with fine-tuned damping values. 

(In the Gazebo link: Scroll down to System Parameters section)