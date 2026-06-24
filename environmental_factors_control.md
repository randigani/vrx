**Glare:** 
---
In the world .sdf file:  
Find the directional light block for the sun:  
```
   <light type="directional" name="sun">  
     <cast_shadows>true</cast_shadows>  
     <pose>5 20 10 0 0 0</pose>  
     <diffuse>1 0.95 0.9 1</diffuse>  
     <specular>1 1 1 1</specular>  
     <!-- Increase ^ the first 3 numbers (up to 1) for stronger glint -->  
     <attenuation>  
       <range>1000</range>  
       <constant>1.0</constant>  
       <linear>0.00</linear>  
       <quadratic>0.000</quadratic>  
     </attenuation>  
     <direction>-0.4 -0.9 -0.2</direction>  
     <!-- Set the vector ^ so the sun is at a low angle -->  
   </light>
```
These set the sun at an angle and optionally increase glint from the reflecting sun. The first three numbers in <specular> are RGB values, the fourth value has no effect (transparency).

In the <scene> block:  
```
   <scene>  
      <sky></sky>  
      <grid>false</grid>  
      <ambient>0.12 0.12 0.12</ambient>  
      <!-- Decrease ^ these ^  for more glare contrast effect  -->  
      <background>0.6 0.6 0.6</background>  
   </scene>
```
This change makes the shadows more intense, creating the dark glare effect. Like before, the three values are RGB values.

**Wind:**  
---
In the world .sdf file: 
```
   <plugin  
     filename="libUSVWind.so"  
     name="vrx::USVWind">  
     <wind_obj>  
       <name>wamv</name>  
       <link_name>wamv/base_link</link_name>  
       <coeff_vector>.5 .5 .33</coeff_vector>  
     </wind_obj>  
     <!-- Wind -->  
     <wind_direction>240</wind_direction>  
     <!-- in degrees -->  
     <wind_mean_velocity>5.0</wind_mean_velocity>  
     <!-- Change the value ^ for wind velocity -->  
     <var_wind_gain_constants>0</var_wind_gain_constants>  
     <var_wind_time_constants>2</var_wind_time_constants>  
     <random_seed>10</random_seed>  
     <!-- set to zero/empty to randomize -->  
     <update_rate>10</update_rate>  
     <topic_wind_speed>/vrx/debug/wind/speed</topic_wind_speed>  
     <topic_wind_direction>/vrx/debug/wind/direction</topic_wind_direction>  
   </plugin>
```
[VRX exposes](https://github.com/osrf/vrx/wiki/wind_params_tutorial) a bunch of parameters for wind simulation. Change wind direction, velocity, modulation, etc. 

**Waves:** 
---
In the world .sdf file:  
```
   <plugin filename="libPublisherPlugin.so" name="vrx::PublisherPlugin">  
     <message type="gz.msgs.Param" topic="/vrx/wavefield/parameters"  
              every="2.0">  
       params {  
         key: "direction"  
         value {  
           type: DOUBLE  
           double_value: 0.0  
         }  
       }  
       params {  
         key: "gain"  
         value {  
           type: DOUBLE  
           double_value: 3.0  
         }  
       }  
       params {  
         key: "period"  
         value {  
           type: DOUBLE  
           double_value: 5  
         }  
       }  
       params {  
         key: "steepness"  
         value {  
           type: DOUBLE  
           double_value: 0  
         }  
       }  
     </message>  
   </plugin>
```

Native [VRX plugin](https://github.com/osrf/vrx/wiki/wave_params_tutorial), Gain controls the height of the waves, period controls the wave period. Direction didn’t really seem to do anything (Could just be me), steepness apparently changes the waves shape.

**Ocean Current:**  
---
Add this to wamv_gazebo.urdf.xacro, under the other `<gazebo>` blocks:  
```
 <gazebo>  
   <plugin filename="gz-sim-hydrodynamics-system"  
           name="gz::sim::systems::Hydrodynamics">  
     <link_name>wamv/base_link</link_name>  
     <default_current>0.00001 0.0 0.0</default_current>  
     <!-- /ocean_current topic only works when this is non-zero...?  -->

     <!-- Linear damping -->  
     <xU>-5.0</xU>  
     <yV>-5.0</yV>  
     <zW>-5.0</zW>  
     <kP>-1.0</kP>  
     <mQ>-1.0</mQ>  
     <nR>-1.0</nR>

     <!-- Quadratic damping -->  
     <xUabsU>-100.0</xUabsU>  
     <yVabsV>-100.0</yVabsV>  
   </plugin>  
 </gazebo>
```

This plugin uses [Gazebo’s realistic hydrodynamic physics](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1systems_1_1Hydrodynamics.html) (Scroll down to System Parameters section). You can change the values in `<default_current>`, or with the simulation running enter in the terminal:  

`gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 2.0, y: -0.0, z: 0.0'` 

Change the values in the quotes for desired current velocity in m/s. Don’t set `<default_current>` to `0 0 0` in wamv_gazebo.xacro.urdf, since I found for some reason this causes the above command to lose its effect during simulation. 

Higher damping coefficients correlates to how strongly the current wants to take the USV with it. Therefore, default values (Zero for all coefficients) results in zero hydrodynamic force on the USV.

Small pitfall:  Since this isn’t a VRX-bespoke plugin, the hydrodynamic force is applied to the entire USV mesh rather than just the submersed part of the hull. However, the difference this produces should be negligible at reasonable current speeds. If realism is needed in extreme scenarios, the difference could be mostly compensated for by fine-tuning damping values. 

For a giggle, run the command during a simulation with a resulting current vector of 200m/s or more. 😅
