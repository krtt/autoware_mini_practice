[< Previous practice](../practice_1) -- [**Main Readme**](../README.md) -- [Next practice >](../practice_3/)

# Practice 2 - Localizer

Localizer determines vehicle position and speed. The position is the result of measurements done by the [GNSS - Global Navigation Satellite System]((https://en.wikipedia.org/wiki/Satellite_navigation)). Speed can be determined from consecutive locations (shift in location and difference in the time). More sophisticated systems (like Novatel SPAN system, for example [Novatel PwrPak7D](https://novatel.com/products/receivers/enclosures/pwrpak7d)) include also IMU measurements to improve the accuracy of GNSS positioning.

In this practice, we will use logged data from the car saved in the rosbag and provided in the `common` package. The GNSS log is recorded in the `/novatel/oem7/inspva` topic:
* location is given as latitude and longitude (geographic coordinates in WGS84 system, with [epsg code 4326](https://epsg.io/4326))
* velocity has three components (east, north and up).

The localizer node must take the latitude and longitude and convert them to UTM coordinates for zone 35N (the cartesian coordinate system with Universal Transverse Mercator projection and having the [epsg code of 25835](https://epsg.io/25835)) so that the car can be localized on the map.

#### Additionally provided:

##### in `practice_2`
* `/nodes/planning/waypoint_saver.py` - node for waypoint saving
* `/launch/practice_2.launch` - a launch file that should run without errors at the end of the practice using your localizer node and also launch waypoint recording
* `/rviz/practice_2.rviz` - rviz config file for visualizing the topics
* `/config/localization.yaml` - some parameter values for localizer

##### in `common`
* `/common/data/bags/ride_14_minimal.bag` - filtered rosbag that contains GNSS log with latitude and longitude data
* `/common/data/trajectories` - folder for waypoint files and one example file


### Expected outcome

* Understanding what does it mean to localize a car on the map
* Your localizer node will convert the measured location from the GNSS system to map coordinates
* As a result, we are able to save waypoint coordinates into a csv file


## 1. Create localizer node

We will create a [subscriber node](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node). The node will get access to the data when we will [play a rosbag](https://wiki.ros.org/rosbag/Commandline#rosbag_play). Rosbag is a recorded dataset, and when playing the rosbag, all the recorded topics are published according to the recorded timestamps; that is how our subscriber will get access to necessary GNSS data.

##### Instructions
1. Create a new file called `localizer.py` inside `~/autoware_mini_practice/src/practice_2/nodes/localization`
2. Copy the following code as a starting point to your node. What to note:
   - See how the node is organized as a class - it is a good practice to agree on some common style when writing a code. Let this be an example of current good practice
      - ROS node is created in [`if __name__ == '__main__':`](https://docs.python.org/3/library/__main__.html#idiomatic-usage) block and then class method `run()` is called
      - organize the code in meaningful groups in class init method 
      - other code should be inside callbacks and functions
   - Go through the necessary imports and note different message types
   - See how subscribers and publishers are created
3. There is an empty callback: `transform_coordinates` - replace `pass` with printing the coordinates `print(msg.latitude, msg.longitude)`

```
#!/usr/bin/env python3

import rospy
import math
from pyproj import CRS, Transformer, Proj
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped

class Localizer:
    def __init__(self):

        # Parameters
        self.undulation = rospy.get_param('/undulation')
        utm_origin_lat = rospy.get_param('/utm_origin_lat')
        utm_origin_lon = rospy.get_param('/utm_origin_lon')

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.transform_coordinates)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
        self.br = TransformBroadcaster()

    def transform_coordinates(self, msg):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('localizer')
    node = Localizer()
    node.run()
```

##### Validation
* As a reminder lets run first all the nodes manually
* In terminal 1: `roscore`
* In terminal 2: `rosbag play --clock ride_14_minimal.bag` - need to be in the same folder where the rosbag is or enter it with the relative path. Bag files are under `common` package `/data/bags`.
* In terminal 3: run the localizer node (`python localizer.py`) and see if the coordinates printed out are roughly similar to the following:

```
latitude:  58.377320927441524  longitude:  26.73093522060624
latitude:  58.37732063697093  longitude:  26.730934613622136
latitude:  58.377320343735214  longitude:  26.730933999096404
...
```
* Close all runnning processes in terminals
* run `roslaunch practice_2 practice_2.launch`
   - we should see the same coordinates printed out in the console
   - all necessary things like playing the rosbag, running the localizer node, and opening rviz with visualization are included in the launch file.


## 2. Convert coordinates

Latitude and longitude are geographic coordinates with the coordinate system name "wgs84". and we saw them being printed out in the previous task.

Our localizer needs to transform these coordinates to UTM zone 35N ([epsg code: 25835](https://epsg.io/25835)), because the map we are using has these coordinates. Additionally, we have defined a custom origin point near the Delta building that needs to be subtracted from transformed coordinates.

For coordinate transformations, we use the [pyproj](https://pyproj4.github.io/pyproj/stable/) library.

##### Instructions
1. Create the coordinate [Transformer](https://pyproj4.github.io/pyproj/stable/api/transformer.html) using the defined [CRS](https://pyproj4.github.io/pyproj/stable/api/crs/crs.html) objects.
2. Use the Transformer to transform the origin point (so that it can be later subtracted from the transformed coordinates).
3. Both of the previous should be done only once in the class initialization 

```
# create coordinate transformer
self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)
```
4. `self.transformer` can be used also inside the callback to transform the coordinates from INSPVA message
5. Add also subtracting the transformed origin coordinates and replace the previous printout in the callback with the current results


##### Validation
* run `roslaunch practice_2 practice_2.launch`
* output should be similar to this:

```
x:  269.20194537803764 y:  -894.3992087505758
x:  269.1676822404843 y:  -894.4305393118411
x:  269.1329317893251 y:  -894.4622026216239
...
```

## 3. Publish `current_pose`

As a next step, we must publish transformed coordinates to a `current_pose` topic. The message type for this topic is [PoseStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html). When you look at the message definition, it contains:
   * [Header message](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html) that has
      * seq - sequence id, int number that is increasing with every sent message
      * stamp - contains the timestamp of the message of type **time**
      * frame_id - reference frame name. Since we will be sending coordinates in a map frame, we need to add a `map` there
   * [Pose message](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Pose.html) consists of two parts, each referring yet to another message type:
      * `position` (message type [geometry_msgs/Point](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Point.html))
      * `orientation` (message type [geometry_msgs/Quaternion](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Quaternion.html))

Let's look separately how we can get the `position` and `orientation`.

##### `position`
* `position` is a [Point](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Point.html) message
   - `position.x` and `position.y` are the transformed coordinates with subtracted origin coordinates
   - for `position.z` we need to take `msg.height` and subtract undulation (parameter)

##### `orientation`
* `orientation` is [Quaternion](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Quaternion.html) message and [quaternion](https://en.wikipedia.org/wiki/Quaternion) is a way to represent angles with respect to coordinate axis aka orientation.
* From INSPVA we have azimuth - angle from North in clockwise direction
* We will be ignoring the car's roll and pitch angles currently
* Essentially, we need to convert the azimuth angle to Quaternion, and then we can assign it as `orientation` in the PoseStamped message. There are a couple of things we need to do to achive that:
   - INSPVA azimuth is in the wgs84 system (ellipsoidal system and the North direction is always along the meridian pointing to the North); we need to correct it for the UTM zone 35N coordinate system (cartesian coordinates on the planar surface where the North direction is taken from the central meridian).
   - The correction depends on location and is also known as meridian convergence. This can be found using the following code line with [get_factors](https://pyproj4.github.io/pyproj/stable/api/proj.html#pyproj.Proj.get_factors) method (see the example code line below)
   - Next thing is we need to convert angle from azimuth (**clockwise (CW)** angle from North - **y axis**) to **counterclocḱwise (CCW)** angle from **x-axis** (this is how angles are usually represented in ROS). Let's call this angle **yaw**. For that, the function convert_azimuth_to_yaw() is provided in the code section
   - And finally, we can convert the angle to Quaternion; you can use the `quaternion_from_euler` for that

```
# calculate azimuth correction
azimuth_correction = (self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence)

# convert azimuth to yaw angle
def convert_azimuth_to_yaw(azimuth):
    """
    Converts azimuth to yaw. Azimuth is CW angle from the North. Yaw is CCW angle from the East.
    :param azimuth: azimuth in radians
    :return: yaw in radians
    """
    yaw = -azimuth + math.pi/2
    # Clamp within 0 to 2 pi
    if yaw > 2 * math.pi:
        yaw = yaw - 2 * math.pi
    elif yaw < 0:
        yaw += 2 * math.pi

    return yaw

# Convert yaw 
x, y, z, w = quaternion_from_euler(0, 0, yaw)
orientation = Quaternion(x, y, z, w)
```

##### Instructions
1. Create the PoseStamped() message
2. Use previously provided code to calculate x, y, z values and orientation
3. `stamp` should be taken from the INSPVA message header
4. `frame_id` should be `"map"`
5. Publish `current_pose`

```
# publish current pose
current_pose_msg = PoseStamped()
current_pose_msg.header.stamp = 
current_pose_msg.header.frame_id = 
current_pose_msg.pose.position.x = 
current_pose_msg.pose.position.y = 
current_pose_msg.pose.position.z = 
current_pose_msg.pose.orientation = 
self.current_pose_pub.publish(current_pose_msg)
```


##### Validation
* run `roslaunch practice_2 practice_2.launch`
* `rostopic echo /current_pose` should see messages published similar to this:

```
header: 
  seq: 1420
  stamp: 
    secs: 1698739108
    nsecs: 206858580
  frame_id: "map"
pose: 
  position: 
    x: 206.55750858492684
    y: -861.8742412924767
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.9144929346097534
    w: 0.4046018691860446
```

## 4. Publish `current_velocity`

Velocity is represented in `/novatel/oem7/inspva` messages with three components: north_velocity, east_velocity and up_velocity. We will use only the North and east components and take their norm.

Message type in `current_velocity` is [TwistStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistStamped.html), and we are going to write the calculated velocity to `message.twist.linear.x` and say that `message.header.frame_id` equals to `base_link`. `base_link` is a common way to name the robots/cars main reference frame where the x axis points forward. That is why we are calculating the norm and writing it only to `twist.linear.x`.

##### Instruction
1. Calculate the velocity as norm of the `north_velocity` and `east_velocity`
2. Create TwistStamped message
3. Create the message 
   * Set frame as `"base_link"` 
   * take a time stamp from INSPVA message
   * assign velocity to `msg.twist.linear.x`
4. Publish to `current_velocity`


## 5. Create and publish transform

* As a last thing, our localizer needs to publish the transform between the `map` frame and the `base_link` frame. For that, you need to use [TransformBroadcaster() from tf2_ros](https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29) library. That is already imported and created.
* It uses message type: [geometry_msgs/TransformStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html)

1. Create the TransformStamped message
   - `frame_id` is `map`
   - `child_frame_id` should be `base_link`
   - timestamp should come from the INSPVA message
2. Transform consists of translation and rotation that are basically current_pose and orientation
3. Use `sendTransform()` to publish the transform (TransformStamped message)


```
# create a transform message
t = TransformStamped()

# fill in the transform message - t

# publish transform
self.br.sendTransform(t)

```

##### Validation 
* `roslaunch practice_2 practice_2.launch`
* Rviz should open with a similar visualization. Location information is played from the rosbag, your localization node transfrms them into UTM Zone 35N coordinates, and also waypoint recording is launched.

![saved_waypoints_img](doc/saved_waypoints.png)

* Big red arrow in front is the location of `current_pose`
* Smaller arrows depict the recorded waypoints, waypoint itself is at the beginning of the arrow and the arrow shows the orientation
* White number on top of the waypoints shows speed in km/h
* The color of the waypoint arrow shows the blinker
   - blue - right turn
   - green - straight
   - red - left turn

If you open the recorded csv file then the contents should look like this:
```
x,y,z,yaw,velocity,change_flag,steering_flag,accel_flag,stop_flag,event_flag
268.60110718221404,-894.9431115202606,0.0,-138.0360894928333,2.7723594859024563,0,0,0,0,0
267.84536724333884,-895.635495165363,0.0,-138.0351092682738,3.2647263562356508,0,2,0,0,0
267.0734381066868,-896.34648047667,0.0,-138.07574386142207,3.7157067324175,0,2,0,0,0
266.3270506322733,-897.026044420898,0.0,-137.97208005347792,4.065691569884165,0,2,0,0,0
265.5763419162831,-897.7114233719185,0.0,-137.83120457759426,4.3996406220710504,0,2,0,0,0
...
```
* `x,y,z` - waypoint coordinates in map frame
* `yaw` - yaw angle in degrees (CCW from x-axis). A negative angle "turns the direction around"
* `velocity` - speed of the car
* `steering_flag` - blinker information
* other columns are currently not used


## 6. Launch file arguments

If you open the launch file, you can see three arguments in the beginning
* `bag_file` - specify the bag file name
* `interval` - define spacing between waypoints
* `waypoints_file` - define output waypoints file name

These arguments can be specified when you run the launch file. The tab key is beneficial here because of auto-complete. Try to enter the same roslaunch command `roslaunch practice_2 practice_2.launch` by hitting the tab key at various steps.

Additionally, enter the tab key at the end and see the arguments appear. For example, try the command with arguments `roslaunch practice_2 practice_2.launch interval:=5 waypoints_file:=waypoints_5m.csv`
