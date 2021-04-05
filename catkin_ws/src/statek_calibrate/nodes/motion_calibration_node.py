#!/usr/bin/env python
import time
import rospy
from statek_msgs.srv import VelocityTest

def type_float(text):
    while(True):
        try:
            return float(raw_input(text))
        except:
            print("Type floating point value!")

def type_int(text):
    while(True):
        try:
            return int(raw_input(text))
        except:
            print("Type integer value!")

def velocity_test(namespace, service_name, test_time):
    print("Starting velocity test...")
    time.sleep(1)
    print("3")
    time.sleep(1)
    print("2")
    time.sleep(1)
    print("1")
    time.sleep(1)

    full_service_name = "/" + namespace + service_name
    rospy.wait_for_service(full_service_name)
    try:
        velocity_test = rospy.ServiceProxy(full_service_name, VelocityTest)
        velocity_response = velocity_test(test_time)
        print("Test finished, found velocity is %f radians per second. If this value is bigger than around 4 that could mean that encoders have gone wrong." % velocity_response.velocity)
    except rospy.ServiceException as e:
        print("Test failed: %s" % e)
        return -1
    return velocity_response.velocity

rospy.init_node("motion_calibration_node", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
max_velocity_service_name = rospy.get_param("~max_velocity_service_name", "/real_time/motors/max_velocity_test")
test_time = rospy.get_param("~test_time", 5000)

print("Before performing this test make sure that the UAV is lifted a bit so it won't move when wheels will start spinning!")
raw_input("Press anything to proceed or Ctrl + C to exit.")

wheel_radius = type_float("Type wheel radius [m]: ")
distance_between_wheels = type_float("Type distance between front left and right wheel centers [m]: ")
wheel_max_angular_velocity = velocity_test(statek_name, max_velocity_service_name, test_time)
max_linear_velocity = wheel_max_angular_velocity * wheel_radius
max_angular_velocity = max_linear_velocity / (distance_between_wheels / 2)

loop_rate = type_int("Type control loop update rate in milliseconds: ")

result = """# wheel config
wheel_radius: {wheel_radius} # In [m].
wheel_max_angular_velocity: {wheel_max_angular_velocity} # How fast wheels can rotate in [rad/s].

# kinematic config
distance_between_wheels: {distance_between_wheels} # In [m].
max_linear_velocity: {max_linear_velocity} # How fast UAV can move forward in m/s defined as wheel_radius*wheel_max_angular_velocity.
max_angular_velocity: {max_angular_velocity} # How fast UAV can rotate around it's center in [rad/s] defined as max_linear_velocity / (distance_between_wheels / 2).

# closed loop control
loop_update_rate_ms : {loop_rate}
left_motor_pid: {left_motor_pid} # Kp, ki, kd.
right_motor_pid: {right_motor_pid}
""".format(wheel_radius=wheel_radius, 
           wheel_max_angular_velocity=wheel_max_angular_velocity,
           distance_between_wheels=distance_between_wheels,
           max_linear_velocity=max_linear_velocity,
           max_angular_velocity=max_angular_velocity,
           loop_rate=loop_rate,
           left_motor_pid=[0,0,0],
           right_motor_pid=[0,0,0])

print("\nGenerated yaml file:")
print(result)

ok = False
while(not ok):
    response = raw_input("Save? [y/N] ")
    response = response.lower().strip()
    if response == "y":
        ok = True
        print("Config saved.")
    elif response == "n" or response == "":
        ok = True
        print("Config ignored.")
    else:
        print("Unknown response.")

print("Calibrator finished.")
