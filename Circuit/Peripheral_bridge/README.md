# Zeabus-Elec-2017-Peripheral_bridge
## Use relative path to ../Kicad-Libraries for specific components
Bridge circuit for several peripherals. It communicates to NUC via a USB (through USB hub). The specifications are:
1. Use 5V power from USB connector
2. Has 2 **isolated** RS-232 ports (use ADM3251E as the tranceiver/receiver) for:
 * DSP board
 * DVL board
3. Has an analog input and a 5V power-supply for a presure sensor.
4. Has 8-channel digital output with **opto-isolator** for solinoid valves.

Because this module has many sub-modules, there are several subscriber and publisher topics links to this module. Sub-modules are:
1. Solenoid-valve switches
2. Barometer
3. Rs-232 Communication channel 1
4. Rs-232 Communication channel 2

The diagram of these sub-modules (each of them can be cosidered as a node) is as:

<pre>
+------------------+
|                  |-------- /zeabus/elec/barometer ------&gt;
| Barometer        |
|                  |-----+
+------------------+     |
                         |
+------------------+     |
|                  |&lt;------ /zeabus/elec/solenoid_sw -----
| Solenoid Switches|     |
|                  |-----+
+------------------+     |
                         +--- /zeabus/elec/hw_error ------&gt;
+------------------+     |
|                  |-----+ 
| RS-232 Channel 1 |&lt;------ /zeabus/elec/comm1/send ------
|                  |---------- /zeabus/elec/comm1/recv ---&gt;
+------------------+     |
                         |
+------------------+     |
|                  |-----+
| RS-232 Channel 2 |&lt;------ /zeabus/elec/comm2/send ------
|                  |---------- /zeabus/elec/comm1/recv ---&gt;
+------------------+
</pre>

The topic **/zeabus/elec/hw_error** is commonly used among all nodes and modules (including the Power Distributor) to announce error messages of the modules. 
