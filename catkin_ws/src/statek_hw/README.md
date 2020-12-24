# statek_hw
Launch file to launch all attached sensors and actuators both real time and non real time. Also launches tfs.

* **launch files**
  * **statek.launch** - Launch all sensors and actuators attached to the platform. </br>
  
    | args | description |
    |-|-|
    | statek_name | Used to create namespaces for sensors, actuators and tfs, i.e **<statek_name>/scan**. |

    | result |
    |-|
    | Launch file will run all sensor, actuator and tf publisher nodes. |
