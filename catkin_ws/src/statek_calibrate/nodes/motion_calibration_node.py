#!/usr/bin/env python
import time
import sys
import numpy as np
import scipy.optimize as opt
import rospy
import rospkg
from statek_msgs.srv import RunVelocityTest
from statek_msgs.srv import RunVelocityTestRequest
from statek_msgs.srv import RunModelIdentification
from statek_msgs.srv import RunModelIdentificationRequest
from statek_msgs.srv import RunImuCalibration
from statek_msgs.srv import RunImuCalibrationRequest

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

def to_squared_array_string(items):
    result = "[" + ', '.join(map(str, items)) + "]"
    return result

def send_motor_params(namespace, param_service, params):
    full_service_name = "/" + namespace + param_service
    rospy.loginfo("Sending parameters to %s" % (full_service_name))
    rospy.wait_for_service(full_service_name)
    try:
        service = rospy.ServiceProxy(full_service_name, SetMotorParams)

        req = SetMotorParamsRequest()
        req.loop_update_rate_ms = params["loop_update_rate_ms"]
        req.wheel_max_angular_velocity = params["wheel_max_angular_velocity"]
        req.kp = params["kp"]
        req.ki = params["ki"]
        req.kd = params["kd"]

        res = service(req)
        if res.success == False:
            rospy.logwarn("Failed to send parameters.")
            return False
    except:
        rospy.logwarn("Exception occured! Failed to send parameters.")
        return False
    return True

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
        service = rospy.ServiceProxy(full_service_name, RunVelocityTest)

        req = RunVelocityTestRequest()
        req.time_ms = test_time

        service_response = service(req)

        if service_response.success == True:
            print("Test finished, found velocity is %f radians per second." % service_response.velocity)
            return service_response.velocity
        else:
            print("Test failed.")
            return -1

    except rospy.ServiceException as e:
        print("Test failed: %s" % e)
        return -1

def step_response_identification(namespace, service_name, test_time):
    print("Starting step response...")
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
        service = rospy.ServiceProxy(full_service_name, RunModelIdentification)

        req = RunModelIdentificationRequest()
        req.identification_time_ms = test_time

        service_response = service(req)

        if service_response.success == True:
            print("Test finished.")
            return {
                "sampling_time": service_response.sampling_time,
                "samples": service_response.samples
            }
        else:
            print("Test failed.")
            return -1
    except rospy.ServiceException as e:
        print("Test failed: %s" % e)
        return -1

def imu_calibration(namespace, service_name):
    print("Starting IMU calibration...")
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
        service = rospy.ServiceProxy(full_service_name, RunImuCalibration)

        req = RunImuCalibrationRequest()
        service_response = service(req)

        if service_response.success == True:
            print("Test finished.")
            return {
                "acc_bias": service_response.acc_bias,
                "gyro_bias": service_response.gyro_bias,
                "mag_bias": service_response.mag_bias,
                "mag_scale": service_response.mag_scale,
            }
        else:
            print("Test failed.")
            return -1
    except rospy.ServiceException as e:
        print("Test failed: %s" % e)
        return -1

def get_model_state(order, samples, current_sample, input_val):
    states_to_add = order - 1
    state = []

    # add order - 1 previous samples
    # starting from the most recent
    for some_previous_sample in range(1, order):
        # in case there is no enough samples
        if current_sample - some_previous_sample < 0:
            break
        state.append(samples[current_sample - some_previous_sample])
    
    # if there is no enough samples then
    # fill with zeros
    if len(state) < states_to_add:
        for k in range(states_to_add - len(state)):
            state.append(0)

    # add step input
    state.append(input_val)

    state = np.matrix(state)
    state = np.transpose(state)

    return state

def rls(order, data):
    print("Starting model identification...")
    samples = data["samples"]

    sigma = 10000
    lbda = 0.99
    
    w = np.zeros((order,1))
    P = sigma * np.eye(order)
    for k in range(len(samples)):
        state = get_model_state(order, samples, k, 1)
        alfa = samples[k] - np.dot(np.transpose(state), w)
        g = np.dot(P, state) * 1 / (lbda + np.dot(np.dot(np.transpose(state), P), state))
        P = 1 / lbda * P - np.dot(np.dot(g, np.transpose(state)), 1 / lbda * P)
        w = w + np.multiply(alfa, g)

    print("Model found: ")
    print(np.transpose(w))
    return w

class PI:
    def __init__(self, kp, ki, dt):
        self.previous_control_val = 0
        self.previous_error = 0
        self.integral = 0

        self.kp = kp
        self.ki = ki
        self.dt = dt

    def read(self, error):
        self.integral = self.integral + error * self.dt
        derivative = (error - self.previous_error) / self.dt
        control_val = self.kp * error + self.ki * self.integral

        sat = False
        if control_val > 1:
            if((error > 0 and control_val > 0) or (error < 0 and control_val < 0)):
                self.integral = self.integral - error * self.dt
            control_val = 1
            sat = True
        elif control_val < -1:
            if((error > 0 and control_val > 0) or (error < 0 and control_val < 0)):
                self.integral = self.integral - error * self.dt
            control_val = -1
            sat = True

        self.previous_error = error
        self.previous_control_val = control_val

        return {"control_val": control_val, "saturation": sat}

def simulate_object(pi_params, *args):
    steps = 100

    order = args[0]
    model = args[1]
    sampling_time = args[2]

    reg = PI(pi_params[0], pi_params[1], sampling_time)
    object_outputs = [0]

    iae = 0
    ise = 0
    penalty = 0
    for i in range(steps):
        error = 1 - object_outputs[i]
        ise += error**2
        iae += np.abs(error)

        result = reg.read(error)
        # Punish PI saturation
        if result["saturation"]:
            penalty += (np.abs(result["control_val"]) - 1)

        control_val = result["control_val"]

        object_output = np.asscalar(np.dot(np.transpose(get_model_state(order, object_outputs, i, control_val)), model))
        
        # Punish for overshoot
        if object_output > 1:
            penalty += (object_output - 1)

        object_outputs.append(object_output)

    itae = np.sum(0.5 * iae + 0.5 * ise)
    return itae + penalty

def tune_pid(order, model, sampling_time):
    print("Starting PI optimizer...")
    solution = opt.least_squares(simulate_object, [0, 0], bounds=([0,0],[10,10]), max_nfev=1000, ftol=None, xtol=None, args=(order, model, sampling_time))
    print("Optimal solution found: {solution}, Performance criteria: {ise}".format(solution=solution.x, ise=solution.cost))
    return np.append(solution.x * 0.4, 0) # Lower the gains to slow down the response

rospy.init_node("motion_calibration_node", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
max_velocity_service_name = rospy.get_param("~max_velocity_service_name", "/real_time/motors/max_velocity_test")
right_motor_step_response_service_name = rospy.get_param("~right_motor_step_response_service_name", "/real_time/motors/right/step_response_identification")
left_motor_step_response_service_name = rospy.get_param("~left_motor_step_response_service_name", "/real_time/motors/left/step_response_identification")
imu_calibration_service_name = rospy.get_param("~imu_calibration_service_name", "/real_time/imu/calibrate")
model_order = rospy.get_param("~model_order", 10)

print("Before performing this test make sure that the UAV is lifted a bit so it won't move when wheels will start spinning!")
raw_input("Press anything to proceed or Ctrl + C to exit.")

# KINEMATIC CONFIG
wheel_radius = type_float("Type wheel radius [m]: ")

distance_between_wheels = type_float("Type distance between front left and right wheel centers [m]: ")

wheel_max_angular_velocity = velocity_test(statek_name, max_velocity_service_name, 10000)
if wheel_max_angular_velocity == -1:
    sys.exit(-1)

wheel_max_angular_velocity *= 0.8

max_linear_velocity = wheel_max_angular_velocity * wheel_radius

max_angular_velocity = max_linear_velocity / (distance_between_wheels / 2)

# CONTROL LOOP TUNING
loop_rate = 30

left_motor_step_response_data = step_response_identification(statek_name, left_motor_step_response_service_name, 600)
if left_motor_step_response_data == -1:
    sys.exit(-1)
left_model = rls(model_order, left_motor_step_response_data)
left_motor_pid = tune_pid(model_order, left_model, left_motor_step_response_data["sampling_time"])

right_motor_step_response_data = step_response_identification(statek_name, right_motor_step_response_service_name, 600)
if right_motor_step_response_data == -1:
    sys.exit(-1)
right_model = rls(model_order, right_motor_step_response_data)
right_motor_pid = tune_pid(model_order, right_model, right_motor_step_response_data["sampling_time"])

# IMU CALIBRATION
print("Now, we'll calibrate IMU. Place the UAV on the ground and make sure that it has a bit of space around as it will spin around during magnetometer calibration.")
raw_input("Press anything to proceed or Ctrl + C to exit.")

imu_update_rate_ms = loop_rate
imu_config = imu_calibration(statek_name, imu_calibration_service_name)

magnetic_declination = type_float("Type magnetic declination in your area as deg.min (i.e from here https://www.magnetic-declination.com/): ")

# ODOM CONFIG
odom_update_rate_ms = loop_rate

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

# imu config
imu_update_rate_ms: {imu_update_rate_ms}
acc_bias: {acc_bias}
gyro_bias: {gyro_bias}
mag_bias: {mag_bias}
mag_scale: {mag_scale}
magnetic_declination: {magnetic_declination}

# odom config
odom_update_rate_ms: {odom_update_rate_ms}
""".format(wheel_radius=wheel_radius, 
           wheel_max_angular_velocity=wheel_max_angular_velocity,
           distance_between_wheels=distance_between_wheels,
           max_linear_velocity=max_linear_velocity,
           max_angular_velocity=max_angular_velocity,
           loop_rate=loop_rate,
           left_motor_pid=to_squared_array_string(left_motor_pid),
           right_motor_pid=to_squared_array_string(right_motor_pid),
           imu_update_rate_ms=imu_update_rate_ms,
           acc_bias=to_squared_array_string(imu_config["acc_bias"]),
           gyro_bias=to_squared_array_string(imu_config["gyro_bias"]),
           mag_bias=to_squared_array_string(imu_config["mag_bias"]),
           mag_scale=to_squared_array_string(imu_config["mag_scale"]),
           magnetic_declination=magnetic_declination,
           odom_update_rate_ms=odom_update_rate_ms)

print("\nGenerated yaml file:")
print(result)

r = rospkg.RosPack()
path = r.get_path("statek_config")
conf_name = "motion_config.yaml"
full_path = path + "/yaml/" + conf_name

ok = False
while(not ok):
    response = raw_input("Save to %s? [y/N] " % full_path)
    response = response.lower().strip()
    if response == "y":
        ok = True
        f = open(full_path, "w")
        f.write(result)
        f.close()
        print("Config saved.")
    elif response == "n" or response == "":
        ok = True
        print("Config ignored.")
    else:
        print("Unknown response.")

print("Calibrator finished.")
