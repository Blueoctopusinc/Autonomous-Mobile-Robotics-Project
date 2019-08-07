import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, Odometrygit
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from tf.transformations import euler_from_quaternion
import actionlib

PI = 3.1415926535897


class objectFinder:
    # Local variables, mainly used for boolean checks and access to data in class functions
    occupancyGrid = None
    x = 0
    y = 0
    z = 0
    resolution = 0
    posex = 0
    posey = 0
    angle = None
    distanceToObject = None
    goalCount = 0
    goals = []
    masked = {}
    quat = transformations.quaternion_from_euler(0, 0, math.radians(90))
    objectFound = False
    inAction = False
    centering = False
    finalAlignment = False
    search = False
    currMask = None
    currColor = None
    skipColor = []
    blueFound = False
    greenFound = False
    redFound = False
    yellowFound = False
    foundColors = []
    stopTurn = False
    objectWayPoints = []
    objectWaypointCount = 0
    currObjectWaypoint = []
    stopTurn = False
    allFound = False

    def __init__(self):
        # class initialisation

        self.bridge = CvBridge()
        # Conversion between ROS camera messages to image formats
        cv2.startWindowThread()
        rospy.init_node('test')
        ###########################PUBLISHERS########################################################
        # Publisher to publish simple movemement commands in the form of linear and angular velocities
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        ##########################SUBSCRIBERS########################################################
        # Subscribes to the turtlebots camera returning the message to imageCB
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageCB)
        # Subscribes to odometry information so the robots current position can be used
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.poseCB)
        # Subscribes to the psuedo laser, psuedo as the turtlebot does not contain a laser scanner but is inferred from the Pointcloud
        # which is a combination of the RGB image + depth in the form RGBD (Red, Green, Blue, Depth), could make use of the pointcloud
        # but aperture's of the RGB and depth camera are different, not all RGB values have a depth would need an averaging filter to
        # be passed over
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scanCB)
        # Subscribes to the map topic to get the occupancy grid
        self.mapSub = rospy.Subscriber("/map", OccupancyGrid, self.callback)
        # Twist object used as a template for twist messages for linear/angular velocities
        self.twist = Twist()
        # Creates a simple action client to publish move base commands, this allows for the use of the ROS navigation stack
        # and utilize the global and local path planner
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Waits for action client to become available to avoid pushing navigation goals before it is initialized
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("action server not available")
        print(wait)
        # Function to issue the move navigation goal
        self.movebase_client()
        # Main class that manages the core logic loop
        self.classMain()
        rospy.spin()

    def classMain(self):
        # Runs the main loop until all of the objects have been found
        while self.allFound == False:
            # Constantly checks the current frame (image) contains one of the objects through colour thresholding
            check = self.checkimage()
            if check:
                # If an object has been seen, aligns the camera with the object and check the distance (laserscan has a limit of 5m)
                self.client.cancel_all_goals()
                inDistance = self.align()
                print("aligned with object")
                if inDistance:
                    # Gets the current X Y Z pose of the robot and it's distance from the detected object
                    currAngle, currPosX, currPosY, currDis = self.angle, self.posex, self.posey, self.distanceToObject
                    # Passes the values to the helper function that converts Polar co-ordinates to Cartesian using trigonometry
                    x, y = self.polar2cart(currPosX, currPosY, currAngle, currDis)
                    # Initializes an empty list (replacing existing points if they are lingering) to store the 4 co-ordinates around the object
                    self.objectWayPoints = []
                    # Generates 4 co-ordinates around the object at a distance of +-0.8m away in both X and Y axis
                    self.objectWayPoints = self.objectWayPointGen(x, y)

                    # Create movebase goal to first point around object
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    print(len(self.goals))
                    goal.target_pose.pose = Pose(Point(self.objectWayPoints[self.objectWaypointCount][0],
                                                       self.objectWayPoints[self.objectWaypointCount][1], 0),
                                                 Quaternion(self.quat[0], self.quat[1], self.quat[2], self.quat[3]))
                    print(goal)
                    self.search = True
                    print(self.search)
                    # Sends goal the the movebase client
                    self.client.send_goal(goal, self.done_cb_object, self.active_cb_object, self.feedback_cb_object)
                    # Boolean condition that passes until the search has been complete
                    while self.search == True:
                        pass
                    # Once search has completed, returns back to the goal it was previously navigating to before searching for object
                    self.movebase_client()
                    print("in search loop")
                else:
                    # If the distance to the object is too far, continues to the waypoint it was navigating to
                    self.movebase_client()

    ############################################# CALLBACK FUNCTIONS ###############################################################

    # Callback function for odometry data, used to store the robots current pose in a class variable
    def poseCB(self, data):
        self.posex = data.pose.pose.position.x
        self.posey = data.pose.pose.position.y
        # Convertes quaternions to euler angles for use in trigonometry (only the Yaw/Rotation in relation to Z axis needed)
        currQuat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(currQuat)
        self.angle = yaw
        # Callback function for laser scan (interpreted from poincloud as noted above)

    def scanCB(self, scan):
        scan.angle_max = 1
        scan.angle_min = -1
        # Get the distance of the centerpoint of the laser scan, used at it will be at the same angle as the robots pose in the Z axis
        distance = scan.ranges[len(scan.ranges) / 2]
        self.distanceToObject = distance

    # Callback for the image subscriber
    def imageCB(self, rgb):
        # Creates an openCV window to see the robots viewpoint
        cv2.namedWindow("window", 1)

        rgbImage = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
        # Converts the RGB image to HSV to decouple the hue, saturation and value and easier to perform accurate colour slicing
        hsvImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2HSV)

        # Stores the height and width of the image, used for splitting the image in half to avoid the robot detecting poles behind small objects as laser
        # scan would be innacurate
        self.h, self.w, self.d = hsvImage.shape
        self.searchHalf = self.h / 2
        # Upper and lower bounds for colour slicing
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])

        lower_green = np.array([50, 150, 100])
        upper_green = np.array([90, 255, 255])

        lower_red = np.array([0, 150, 30])
        upper_red = np.array([8, 255, 150])

        lower_yellow = np.array([27, 195, 100])
        upper_yellow = np.array([50, 255, 255])

        bMask = cv2.inRange(hsvImage, lower_blue, upper_blue)
        gMask = cv2.inRange(hsvImage, lower_green, upper_green)
        rMask = cv2.inRange(hsvImage, lower_red, upper_red)
        yMask = cv2.inRange(hsvImage, lower_yellow, upper_yellow)
        # Threshhold all the colours to create masked images for colour detection
        # and store them in class variables for later use
        self.masked["blue"] = bMask
        self.masked["green"] = gMask
        self.masked["red"] = rMask
        self.masked["yellow"] = yMask
        # Output the image to the OpenCV window
        cv2.imshow("window", hsvImage)
        cv2.waitKey(1)

    def callback(self, data):

        self.x = data.info.origin.position.x
        self.y = data.info.origin.position.y
        self.z = data.info.origin.position.z
        # Get origin position, used for offset when calculating x, y co-ords as map origin and robots origin are different

        onedMap = data.data
        width = data.info.width
        height = data.info.height
        resolution = data.info.resolution
        # Store data about the maps shape and resolution scale, used to turn map co-ords into world co-ords
        self.resolution = resolution
        numpyArr = np.asarray(onedMap)
        # Reshape the 1D array into a 2D array so co-ordinates can be generated (originally in Row Major format)
        b = np.reshape(numpyArr, (height, width))
        self.occupancyGrid = b

        # Saves the occupancy grid in a class variable
        # Occupancy grid values are a range of 0-100 for occupied, -1 for unknown. Map given only had 100, -1 or 0 changes 100 and -1 values to 255
        # to represent a binary image
        for i in range(height):
            for j in range(width):
                if b[i][j] == 100:
                    b[i][j] = 255
                elif b[i][j] == -1:
                    b[i][j] = 255

        cv2.imwrite("map.jpg", b)
        # Kernels for morpholigical operations on the map
        kernel = np.ones((8, 8), np.uint8)
        erosionKernel = np.ones((4, 4), np.uint8)

        # Dilates the map to accentuate it's features, also to minimise any nose
        dilatedMap = cv2.dilate(b.astype(np.uint8), kernel, iterations=2)
        cv2.imwrite("dilated.jpg", dilatedMap)
        # Erodes the map to reduce it,improves the clarity of the edges for canny edge detection
        erosion = cv2.erode(dilatedMap, erosionKernel, iterations=2)
        cv2.imwrite("eroded.jpg", erosion)
        # Performs canny edge detection on the eroded map to get the inner edge for hough line detection

        edgesDetected = cv2.Canny(erosion, 100, 200, 3)
        cv2.imwrite("canny.jpg", edgesDetected)
        # Empty array will hold the results of hough line detection
        lineIMG = np.zeros((height, width), np.uint8)
        # Hough lines detection on the canny edge detected copy of the map
        lines = cv2.HoughLinesP(edgesDetected, rho=1, theta=np.pi / 180, threshold=50, minLineLength=86, maxLineGap=60)
        # Iterates through the lines found through Hough Line Detection
        for line in lines:
            # Iterates through the endpoints for each of the lines

            for x1, y1, x2, y2 in line:
                # Calculates the gradient (different in x/different in x)
                gradient = (float(y2) - y1) / (float(x2) - x1)
                # Calculates the centerpoint of the line from which it will extend outwards
                centerX = (x1 + x2) / 2
                centerY = (y1 + y2) / 2

                # Calculates the distance between the centerpoint of the line and it's first point (the same for both points as it's calculated
                # From the distance)
                distance1P = math.hypot(centerX - x1, centerY - y1)

                # Helper function for finding the new X, Y coordinates of the line extended by the same value on both endpoints
                newX, newY, newX2, newY2 = self.findPoint(distance1P, centerX, centerY, gradient, 10)
                # Adds a line to the line IMG
                cv2.line(lineIMG, (int(newX), int(newY)), (int(newX2), int(newY2)), (255, 0, 0), 6)

        print(lines)
        # Finds the contours (joined points) in the line image, to seperate the image into different rooms
        im2, contours, hierarchy = cv2.findContours(lineIMG, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Iterates through the detected shapes and colculates the co-ordinate of each ones centerpoint
            M = cv2.moments(contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # Converts the map co-ordinate into a world co-ordinate that can be used by the navigation stacks global planner
            worldX, worldY = self.coordConverter(cx, cy, )
            coords = [worldX, worldY]
            self.goals.append(coords)
            # Draw a circle around it for visualisation purposes
            cv2.circle(lineIMG, (cx, cy), 10, 255, -1)
        self.pointCleaner()

        # Sort the goals by Y axis values (Searches from the bottom of the room to the top), has issues with local planner going
        # Left to right/vice vera and top to bottom, could possibly change navigation stack cost map parameters? outside scope of this project
        sortedGoals = sorted(self.goals, key=lambda x: x[1])
        self.goals = sortedGoals
        cv2.drawContours(im2, contours, -1, (0, 255, 0), 3)
        cv2.imwrite("Gray_Image.jpg", lineIMG)

    # Function to align the robot with the center of the camera so the Z pose can be used as an angle to convert polar to cartesian co-ordinates
    # OpenCV moments function returns features of the thresholded image
    def align(self):
        isAligned = False
        while isAligned == False:
            mask = self.masked[self.currColor]
            # Sets the top half of the mask to 0, stops the robot detecting/aligning with objects behind small objects as the laserscan distance would belong to the blocking object
            mask[self.searchHalf:self.h, 0:self.w] = 0
            M = cv2.moments(mask)
            # If the area of the moment is above 0, calculate the centroid and calculates the distance between the centerpoint and center of the moment
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(mask, (cx, cy), 20, (0, 0, 255), -1)
                err = cx - self.w / 2
                if -float(err) / 100 == 0.01:
                    # If the laser scan is out of range ( max 5m) returns NaN, distance is needed for trig to convert polar to cartesian co-ordinates so stops looking for that colour
                    # until it has reached the next waypoint
                    if math.isnan(float(self.distanceToObject)):
                        print(self.distanceToObject)
                        print("object too far to get distance")
                        self.skipColor.append(self.currColor)
                        print(self.skipColor)
                        self.currColor = None
                        return False
                    else:
                        # Returns true if the distance is non NaN, and used in cart2polar to calculate object co-ords
                        print(self.distanceToObject)
                        return True
                # Publish a twist message along the velocity publisher, rotates towards the center of the object
                if -float(err) / 100 != 0.000:
                    self.twist.angular.z = -float(err) / 100
                    self.cmd_vel_pub.publish(self.twist)

    def alignFinal(self):
        # Same function as above for the most part, only checks the mask indicated by currColor
        isAligned = False
        while isAligned == False:
            mask = self.masked[self.currColor]
            mask[self.searchHalf:self.h, 0:self.w] = 0
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(mask, (cx, cy), 20, (0, 0, 255), -1)
                err = cx - self.w / 2
                if -float(err) / 100 == 0.01:
                    # Checks the distance of the object, if it's less than a meter complete noteFound function which notes if a colour has been found
                    if self.distanceToObject < 1:
                        print(self.distanceToObject)
                        print("Object within 1m")
                        print(self.skipColor)
                        isAligned = True
                        self.noteFound()
                        self.currColor = None
                        return True

                if -float(err) / 100 != 0.000:
                    self.twist.angular.z = -float(err) / 100
                    self.cmd_vel_pub.publish(self.twist)
            else:
                return False

    def checkimageClose(self):
        # Checks if the selected mask has an area greater than 150000, this value is used as a lower area value means the robot was trying to center
        # on too slim a section of the object if partially blocked and marking the distance as the object infront, would also lose sight of objects
        # around corners and get stuck in an infinite loop
        i = self.masked[self.currColor]
        i[self.searchHalf:self.h, 0:self.w] = 0
        M = cv2.moments(i)
        if M['m00'] > 150000:
            print("found object in close search")
            return True
        else:
            return False

    def checkimage(self):
        # same as the function above, however notes the detected color as the current color, use of too different functions so if a color is detected but is in close
        # proximity in another only the first value is taken until it returns to the main search loop
        for index, i in self.masked.items():
            if index not in self.skipColor and index not in self.foundColors:
                i[0:self.searchHalf, 0:self.w] = 0
                M = cv2.moments(i)
                if M['m00'] > 150000:
                    print("at finding")
                    self.currColor = index
                    print(self.currColor)
                    return True
        return False

    # Helper function that checks the current color identified, only from within alignFinal and flips the boolean value indicating a marker has been found
    def noteFound(self):
        print("in noteFound Function")
        if self.currColor == "blue":
            self.blueFound = True
            self.foundColors.append(self.currColor)
            print("Blue found at", self.currObjectWaypoint)
            self.currObjectWaypoint = []
            self.currColor = None
        if self.currColor == "green":
            self.greenFound = True
            self.foundColors.append(self.currColor)
            print("Green found at", self.currObjectWaypoint)
            self.currObjectWaypoint = []
            self.currColor = None
        if self.currColor == "red":
            self.redFound = True
            self.foundColors.append(self.currColor)
            print("Red found at", self.currObjectWaypoint)
            self.currObjectWaypoint = []
            self.currColor = None
        if self.currColor == "yellow":
            self.yellowFound = True
            self.foundColors.append(self.currColor)
            print("Yellow found at", self.currObjectWaypoint)
            self.currObjectWaypoint = []
            self.currColor = None

    # Helper function that generates 4 way points above, below and either side of the object (at a distance of 0.8m)
    def objectWayPointGen(self, x, y):
        print(x, y)
        self.objectWayPoints = []
        toCheck = []
        toCheck.append([x + 0.7, y])
        toCheck.append([x - 0.7, y])
        toCheck.append([x, y + 0.7])
        toCheck.append([x, y - 0.7])
        return toCheck

    # one of two functions that initiate navigation to the goals by sending it to the navigation stack, this one is directly for managing movement to objects as navigation
    # and navigating to object have different internal logic
    def movebase_client_object(self):
        print("starting Navigation")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        print(len(self.goals))
        goal.target_pose.pose = Pose(
            Point(self.objectWayPoints[self.objectWaypointCount][0], self.objectWayPoints[self.objectWaypointCount][1],
                  0), Quaternion(self.quat[0], self.quat[1], self.quat[2], self.quat[3]))
        print(goal)
        # Sends the first object waypoint pose to the navigation stack and begins navigation
        self.client.send_goal(goal, self.done_cb_object, self.active_cb_object, self.feedback_cb_object)
        rospy.spin()

    # Unused active object CB
    def active_cb_object(self):
        rospy.loginfo("Active object goal object")

    # Unused feedback callback
    def feedback_cb_object(self, feedback):
        pass

    # Callback for goal completion, initiates sequence for searching at the goal
    def done_cb_object(self, status, result):

        if status == 2:
            rospy.loginfo("Goal cancelled after starting")
        if status == 3:
            # Checks that all waypoints haven't already been visited
            if self.objectWaypointCount < (len(self.objectWayPoints) - 1):
                myYaw = math.degrees(self.angle) + 3
                # Spins the robot until the object has been seen
                while not self.stopTurn:
                    if math.degrees(self.angle) == myYaw:
                        self.stopTurn = True
                    else:
                        self.twist.angular.z = 0.6
                        self.cmd_vel_pub.publish(self.twist)
                        check = self.checkimageClose()
                        if check:
                            print("check")
                            self.stopTurn = True
                # Aligns the robot with the object and checks the distance
                isFound = self.alignFinal()
                print(isFound)
                if not isFound:
                    # If the object is not found or is too far away, moves the the next waypoint
                    print("not found, moving to next waypoint")
                    self.objectWaypointCount += 1
                    self.skipColor = []
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose = Pose(Point(self.objectWayPoints[self.objectWaypointCount][0],
                                                       self.objectWayPoints[self.objectWaypointCount][1], 0),
                                                 Quaternion(self.quat[0], self.quat[1], self.quat[2], self.quat[3]))
                    self.client.send_goal(goal, self.done_cb_object, self.active_cb_object, self.feedback_cb_object)
                    rospy.loginfo("Goal reached")
                if isFound:
                    # If the object is detected note that it has been found and set the search boolean check to true, exits the loop set in the main class to resume normal navigation
                    self.noteFound()
                    self.stopTurn = False
                    print("Object found, exiting object waypoint navigation")
                    self.search = False
            else:
                # If all waypoints are visited and no object has been detected return back to the main function loop by changing the boolean check to true
                print("visited all object waypoints")
                self.search = False
        if status == 4:
            # Gaols invalid, return to main loop
            rospy.loginfo("Goal pose aborted by action server")
            self.search = False
        if status == 5:
            rospy.loginfo("Goal rejected")
        if status == 8:
            rospy.loginfo("Cancelled before execution")
            print(result)

    def polar2cart(self, x, y, angle, dist):
        # Using trigonometry treating the distance to the object as the hypotenuse and Z pose as the angle calculates a waypoint for the object
        # based on https://www.mathsisfun.com/polar-cartesian-coordinates.html
        print(x, "curr X", y, "curr Y", angle, "curr Angle", dist, "curr Dist")
        newx = (x + math.cos(angle) * dist)
        newy = (y + math.sin(angle) * dist)
        print(newx, newy)
        return newx, newy

        # Helper function for the map callback, used to extend the lines generated from Hough line detection

    def findPoint(self, dis, x1, y1, m, incr):
        fullDistance = dis + incr
        # If the gradient = 0 the line is perpendicular to the X axis, just increase/ decrease X co-ordinates
        if m == 0.0:
            endX = x1 + fullDistance
            endY = y1

            endX2 = x1 - fullDistance
            endY2 = y1
        # If the gradient is infinite the line is perpendicular to the Y axis and, increase/decrease Y co-ordinates
        elif m == float('-inf'):
            endX = x1
            endY = y1 + fullDistance

            endX2 = x1
            endY2 = y1 - fullDistance
        # Calculate the displacement of the line along the X and Y axis given the gradient and add it to the centerpoint to generate new line endpoints
        else:
            dx = (fullDistance / math.sqrt(1 + (m * m)))
            dy = m * dx
            endX = x1 + dx
            endY = y1 + dy

            endX2 = x1 - dx
            endY2 = y1 - dy
        return endX, endY, endX2, endY2

    # Unused callback
    def active_cb(self):
        rospy.loginfo("Active goal")

    # Unused feedback callback
    def feedback_cb(self, feedback):
        pass

    # Callback when the a map goalpoint has been reached
    def done_cb(self, status, result):
        if status == 2:
            rospy.loginfo("Goal cancelled after starting")
        if status == 3:
            # Resets colors marked for skipping due to being too far away
            self.skipColor = []
            # Rotates the robot 360 degrees whilst still looking for colors
            self.rotate()
            if self.goalCount < (len(self.goals) - 1):
                # Check if all navigation points have been visited
                if self.currColor is None:
                    print("not found, moving to next waypoint")
                    self.skipColor = []
                    # Increment goal count to move to next waypoint in sequence
                    self.goalCount += 1
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    print(len(self.goals))
                    goal.target_pose.pose = Pose(Point(self.goals[self.goalCount][0], self.goals[self.goalCount][1], 0),
                                                 Quaternion(self.quat[0], self.quat[1], self.quat[2], self.quat[3]))
                    self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

                rospy.loginfo("Goal reached")
            else:
                print("all waypoint visited")
        if status == 4:
            # If current waypoint cannot be reached, return to the previous waypoint
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(self.goals[self.goalCount - 1][0], self.goals[self.goalCount - 1][1], 0),
                                         Quaternion(self.quat[0], self.quat[1], self.quat[2], self.quat[3]))
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
            rospy.loginfo("Goal pose aborted by action server")

        if status == 5:
            rospy.loginfo("Goal rejected")
        if status == 8:
            rospy.loginfo("Cancelled before execution")
            print(result)

    def rotate(self):

        print("rotating robot")

        # Converting from angles to radians
        angular_speed = 1
        relative_angle = 360 * 2 * PI / 360

        # check for turn direction
        self.twist.angular.z = -abs(angular_speed)

        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        # Begin timer to calculate current angle
        while (current_angle < relative_angle):
            self.cmd_vel_pub.publish(self.twist)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
        # Publish twist until goal angle reached

        # Stops turning once goal angle achieved
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def pointCleaner(self):
        print(len(self.goals))
        del self.goals[7]
        del self.goals[4]
        del self.goals[3]

        # Converts a point in the occupancy grid to a world co-ordinate, adds to the origin as the  map and robot origin are different

    def coordConverter(self, pointX, pointY):
        worldX = (pointX * self.resolution) + self.x
        worldY = (pointY * self.resolution) + self.y
        return worldX, worldY

        # Reverse of the obove function, unused but could be used to check co-ordinates on the occupancy grid before navigation - ran out of time

    def world2cost(self, pointX, pointY):
        costX = (pointX - self.x) / self.resolution
        costY = (pointY - self.y) / self.resolution
        return costX, costY

    # function that builds and sends the first navigation goal to begin the navigation sequence
    def movebase_client(self):
        print("starting Navigation")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        print(len(self.goals))
        goal.target_pose.pose = Pose(Point(self.goals[self.goalCount][0], self.goals[self.goalCount][1], 0),
                                     Quaternion(self.quat[0], self.quat[1], self.quat[2], self.quat[3]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)


objectFinder()