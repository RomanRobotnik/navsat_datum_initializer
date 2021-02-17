#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import math
import utm
import copy
import numpy

import actionlib
from actionlib_msgs.msg import GoalStatus
# Insert here msg and srv imports:
from std_msgs.msg import String
from robot_localization.srv import SetDatum, SetDatumRequest

from std_srvs.srv import Trigger, TriggerResponse
from navsat_datum_initializer.msg import *
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix
from tf import TransformListener
from tf.transformations import quaternion_from_euler


from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error


class NavSatDatumInitializer(RComponent):
    """
    Node to call the datum service of navstat_transform_node
    """

    def __init__(self):
        self.initial_pose = [0, 0, 0]
        self.current_pose = [0, 0, 0]
        self.distance = 0.0
        self.gps_msgs_list = []
        self.min_speed = 0.1
        self.goal_reached = False
        self.t_goal_reached = rospy.Time(0)

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.gps_topic_name = rospy.get_param(
            '~gps_topic_name', 'gps/fix')
        self.cmd_vel_topic_name = rospy.get_param(
            '~cmd_vel_topic_name', 'cmd_vel')
        self.base_frame = rospy.get_param(
            '~base_frame', 'base_link')
        self.fixed_frame = rospy.get_param(
            '~fixed_frame', 'odom')
        self.set_datum_service_name = rospy.get_param(
            '~set_datum_service_name', 'navsat_transform_node/datum')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.tf = TransformListener()

        self.gps_sub = rospy.Subscriber(
            self.gps_topic_name, NavSatFix, self.gps_sub_cb)
        self.add_topics_health(self.gps_sub, topic_id=self.gps_topic_name)

        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic_name, Twist, queue_size=10)

        self.action_server = actionlib.SimpleActionServer('~action', DatumInitializerAction, None, False)
        self.action_server.register_goal_callback(self.action_goal_cb)
        self.action_server.register_preempt_callback(self.action_preempt_cb)

        self.set_datum_service_client = rospy.ServiceProxy(self.set_datum_service_name, SetDatum)
        # self.example_server = rospy.Service(
        #    '~example', Trigger, self.example_server_cb)

        return 0

    def init_state(self):

        if self.check_topics_health() == True:
            self.action_server.start()
            self.switch_to_state(State.READY_STATE)
        else:
            rospy.loginfo_throttle(30, "%s::init_state: waiting for required topics" % (self._node_name))

    def ready_state(self):
        """Actions performed in ready state"""
        if self.check_topics_health() == False:
            if self.action_server.is_active() == True:
                self.send_cmd_vel(0.0)
                self.finish_action(GoalStatus.ABORTED, 'Topics are not being received correctly')
            self.switch_to_state(State.EMERGENCY_STATE)

        if self.action_server.is_active() == True:                  # Action server running
            translation, rotation, common_time = self.get_transform(self.fixed_frame, self.base_frame)
            if translation != None:
                self.current_pose = translation
            else:
                self.send_cmd_vel(0.0)
                msg = 'error getting the transform from %s to %s' % (self.fixed_frame, self.base_frame)
                rospy.logerr_throttle(10, '%s::ready_state: %s' % (self._node_name, msg))
                self.finish_action(GoalStatus.ABORTED, msg)
                return

            self.distance = self.get_distance(self.current_pose, self.initial_pose)

            if self.goal_reached == True:
                if (rospy.Time.now() - self.t_goal_reached).to_sec() >= 2.0:
                    # convert lat,log into utm
                    utm_list = []
                    utm_list_x = []
                    utm_list_y = []
                    for p in self.gps_msgs_list:
                        utm_p = utm.from_latlon(p.latitude, p.longitude)
                        utm_list.append(utm_p)
                        utm_list_x.append(utm_p[0])
                        utm_list_y.append(utm_p[1])
                    #print utm_list_x
                    #print utm_list_y
                    linear_regression = LinearRegression()  # creamos una instancia de LinearRegression
                    # instruimos a la regresión lineal que aprenda de los datos (x,y)
                    linear_regression.fit(numpy.reshape(utm_list_x, (-1, 1)), utm_list_y)
                    # vemos los parámetros que ha estimado la regresión lineal
                    print('w = ' + str(linear_regression.coef_) + ', b = ' + str(linear_regression.intercept_))

                    # Predecimos los valores y para los datos usados en el entrenamiento
                    trained_prediction = linear_regression.predict(numpy.reshape(utm_list_x, (-1, 1)))
                    # Calculamos el Error Cuadrático Medio (MSE = Mean Squared Error)
                    mse = mean_squared_error(y_true=utm_list_y, y_pred=trained_prediction)
                    # La raíz cuadrada del MSE es el RMSE
                    rmse = numpy.sqrt(mse)
                    print('Mean Squared Error (MSE) = ' + str(mse))
                    print('(RMSE) = ' + str(rmse))
                    r2 = linear_regression.score(numpy.reshape(utm_list_x, (-1, 1)), utm_list_y)
                    print('Coeficient of Determination R2 = ' + str(r2))

                    # calc the angle of the vector based on the first and last points
                    x0 = numpy.array([utm_list_x[0]])
                    y0 = linear_regression.predict(x0.reshape(-1, 1))
                    x1 = numpy.array([utm_list_x[-1]])
                    y1 = linear_regression.predict(x1.reshape(-1, 1))
                    diff_x = x1 - x0
                    diff_y = y1 - y0
                    angle = math.atan2(diff_y, diff_x)
                    print('DiffX = %lf, DiffY= %lf. Angle of the vector = %lf (%lf)' %
                          (diff_x, diff_y, angle, math.degrees(angle)))

                    srv = SetDatumRequest()
                    srv.geo_pose.position.latitude = self.gps_msgs_list[-1].latitude
                    srv.geo_pose.position.longitude = self.gps_msgs_list[-1].longitude
                    srv.geo_pose.position.altitude = self.gps_msgs_list[-1].altitude
                    q = quaternion_from_euler(0, 0, angle)
                    srv.geo_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                    self.set_datum_service_client(srv)
                    rospy.loginfo("%s::ready: Set datum correctly" % (self._node_name))
                    self.finish_action(GoalStatus.SUCCEEDED, 'OK', [
                                       mse, r2, srv.geo_pose.position.latitude, srv.geo_pose.position.longitude, srv.geo_pose.position.altitude, angle])

            else:
                if self.distance >= self.goal.command.distance:
                    self.send_cmd_vel(0.0)
                    self.goal_reached = True
                    self.t_goal_reached = rospy.Time.now()
                else:
                    self.send_cmd_vel(self.goal.command.speed)

            self.publish_feedback()
        # Publish topic with data

        # data_stamped = StringStamped()
        # data_stamped.header.stamp = rospy.Time.now()
        # data_stamped.string = self.data.data

        # self.cmd_vel_pub.publish(self.data)
        # self.data_stamped_pub.publish(data_stamped)

    def emergency_state(self):
        """Actions performed in emergency state"""
        if self.check_topics_health() == True:
            self.switch_to_state(State.READY_STATE)
        else:
            rospy.loginfo_throttle(30, "%s::emergency_state: waiting for required topics" % (self._node_name))

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def gps_sub_cb(self, msg):
        self.tick_topics_health(self.gps_topic_name)
        if self.action_server.is_active() == True:
            self.gps_msgs_list.append(msg)
        # rospy.logwarn("Received msg: %lf,%lf", msg.latitude, msg.longitude)

    '''def example_server_cb(self, req):
        rospy.logwarn("Received srv trigger petition.")

        response = TriggerResponse()
        response.success = True
        response.message = "Received srv trigger petition."
        return response
        '''

    def action_goal_cb(self):
        """Action server goal callback

        Accepts the new goal if not command running.  Rejects the incoming
        goal if a command is running
        """
        if self.action_server.is_active() == False:
            self.goal = self.action_server.accept_new_goal()
            if self._state != State.READY_STATE:
                rospy.logerr('%s::action_goal_cb: command not accepted because the component is not READY' %
                             (self._node_name))
                result = InitializerResult()
                result.result.success = False
                result.result.message = 'the component is not READY'
                result.result.code = GoalStatus.REJECTED
                self.action_server.set_aborted(result=result, text=result.result.message)
                return

            # initializing values
            if self.goal.command.distance < 0:
                self.goal.command.distance = fabs(self.goal.command.distance)

            if self.goal.command.speed <= self.min_speed:
                self.goal.command.speed = self.min_speed

            self.distance = 0.0
            self.gps_msgs_list = []
            self.goal_reached = False
            self.t_goal_reached = rospy.Time.now()

            translation, rotation, common_time = self.get_transform(
                self.fixed_frame, self.base_frame)
            if translation != None:
                self.initial_pose = copy.deepcopy(translation)
                self.current_pose = copy.deepcopy(translation)
            else:
                msg = 'error getting the transform from %s to %s' % (self.fixed_frame, self.base_frame)
                rospy.logerr_throttle(10, '%s::action_goal_cb: %s' % (self._node_name, msg))
                self.finish_action(GoalStatus.ABORTED, msg)
                return

        else:
            goal = self.action_server.accept_new_goal()
            rospy.logwarn('%s::action_goal_cb: New command not allowed while another command is running', self._node_name)

    def action_preempt_cb(self):
        """Action server preempt callback


        """
        if self.action_server.is_active():
            self.send_cmd_vel(0)
            self.finish_action(GoalStatus.PREEMPTED, 'PREEMPTED')

        else:
            rospy.logwarn('%s::action_preempt_cb: No command is running', self._node_name)

    def publish_feedback(self):
        """Publishes the updated feedback
        """
        feedback_msg = DatumInitializerFeedback()
        feedback_msg.feedback.remaining_distance = (self.goal.command.distance - self.distance)
        feedback_msg.feedback.gps_coordinates_size = len(self.gps_msgs_list)
        self.action_server.publish_feedback(feedback_msg)

    def finish_action(self, code, msg='', args=[]):
        """Finishes the current action

        Args:
            code : int value codified as GoalStatus object
            msg: string to set the message of the result
        """
        result = DatumInitializerResult()
        result.result.message = msg
        result.result.code = code

        if code == GoalStatus.PREEMPTED:
            result.result.success = False
            self.action_server.set_preempted(result=result, text=result.result.message)
        elif code == GoalStatus.SUCCEEDED:
            result.result.success = True
            result.result.mean_squared_error = args[0]
            result.result.coeficient_of_determination = args[1]
            result.result.latitude = args[2]
            result.result.longitude = args[3]
            result.result.altitude = args[4]
            result.result.orientation = args[5]
            self.action_server.set_succeeded(result=result, text=result.result.message)
        elif code == GoalStatus.ABORTED:
            result.result.success = False
            self.action_server.set_aborted(result=result, text=result.result.message)
        else:
            result.result.success = False
            rospy.logwarn(
                '%s::finish_action: the code passed is not being handled! Setting the action as aborted', self._node_name)
            self.action_server.set_aborted(result=result, text=result.result.message)

    def get_transform(self, from_frame, to_frame, common_time=None):
        '''
            @brief gets the transform between two frames
            @return a transform (translation, rotation, common_time) if ok
            @return (None, None, None) if error
        '''
        try:
            if common_time == None:
                t = self.tf.getLatestCommonTime(
                    from_frame, to_frame)
            else:
                t = common_time

            (translation, rotation) = self.tf.lookupTransform(
                from_frame, to_frame, t)

            return translation, rotation, t

        except Exception as e:
            rospy.logerr_throttle(1, "%s::get_transform: exception: %s" % (self._node_name, e))
            return None, None, None

    def get_distance(self, p1, p2):
        """
            @brief calculates the distance between two points
            @param p1 as [double,double, double]
            @param p2 as [double, double, double ]
            @returns the distance as double
        """

        return math.sqrt(((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))

    def send_cmd_vel(self, linear_speed):
        """
            @brief Sends the linear_speed to the controller
            @param linear_speed as double
        """
        msg = Twist()
        msg.linear.x = linear_speed
        self.cmd_vel_pub.publish(msg)
