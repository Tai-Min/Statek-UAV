# statek_tf
Package with static and dynamic transformations. 

* **launch files**
  * **static_transform_publisher.launch** - Static transform publisher for Statek UAV.

  | args | description |
  |-|-|
  | statek_name | Name of Statek UAV. Used to create namespace for transformations. Default is statek. |

  | results |
  |-|
  | Creates static transform publishers for every static link in Statek UAV. |