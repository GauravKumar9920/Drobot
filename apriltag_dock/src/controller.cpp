#include "controller.h"

using namespace automatic_parking;

void autodock_controller::docking_state_manage()
{
    if (docking_state == "searching")
    {
        searching_state_fun();
    }
    else if (docking_state == "blind")
    {
        blind_state_fun();
    }
    else if (docking_state == "centering")
    {
        centering_state_fun();
    }
    else if (docking_state == "approach")
    {
        approach_state_fun();
    }
    else if (docking_state == "final_approach")
    {
        final_approach_state_fun();
    }
    else if (docking_state == "docked")
    {
        movus_stop();
    }
}

void autodock_controller::set_docking_state(const std::string &new_docking_state)
{
    if (docking_state != new_docking_state)
    {
        last_docking_state = docking_state;
        docking_state = new_docking_state;
        RCLCPP_INFO(get_logger(), "Docking state changed: %s -> %s", last_docking_state.c_str(), docking_state.c_str());
    }
}

void autodock_controller::set_action_state(const std::string &new_action_state)
{
    if (action_state != new_action_state)
    {
        last_action_state = action_state;
        action_state = new_action_state;
        RCLCPP_INFO(get_logger(), "Action state changed: %s -> %s", last_action_state.c_str(), action_state.c_str());
    }
}

void autodock_controller::searching_state_fun()
{
    centering_counter = 0;
    if (action_state == "turning" || action_state == "jogging")
        return;

    if (tag_callback_counter < lost_tag_max)
    {
        set_action_state("count_tag_callbacks");
    }
    else
    {
        tag_callback_counter = 0;
        set_action_state("");
        movus_turn(default_turn * sign(tag_y));
    }
}

void autodock_controller::blind_state_fun()
{
    if (action_state == "turning" || action_state == "jogging")
        return;

    if (in_view)
    {
        movus_stop();
        movus_turn(blind_angle * sign(-tag_y));
    }
    else
    {
        movus_stop();
        movus_forward(fabs(tag_y / 1.5));
        set_docking_state("searching");
    }
}

void autodock_controller::centering_state_fun()
{
    if (action_state == "turning" || action_state == "jogging")
        return;

    if (tag_callback_counter < 1)
    {
        centering_counter++;
        set_action_state("count_tag_callbacks");
        return;
    }

    tag_callback_counter = 0;
    set_action_state("");

    if (centering_counter >= max_center_count)
    {
        RCLCPP_WARN(get_logger(), "Centering failed. Reverting to state: searching");
        set_docking_state("searching");
        return;
    }

    if (in_view)
    {
        if (fabs(pose_set.theta) > pose_set.theta_bounds)
        {
            movus_stop();
            movus_turn(pose_set.theta);
        }
        else
        {
            movus_stop();
            set_docking_state("approach");
        }
    }
}

void autodock_controller::approach_state_fun()
{
    centering_counter = 0;

    if (in_view)
    {
        approach_counter = 0;

        if (action_state == "jogging")
            return;

        if (desire_angle == 0)
        {
            if (fabs(pose_set.theta) > pose_set.theta_bounds)
            {
                RCLCPP_INFO(get_logger(), "Approach angle exceeded: %f", fabs(pose_set.theta));
                set_docking_state("centering");
            }
            else if (fabs(pose_set.distance - finish_distance) < jog_distance)
            {
                movus_stop();
                movus_forward(pose_set.distance - finish_distance);
                set_docking_state("final_approach");
            }
            else
            {
                movus_forward(jog_distance);
            }
        }
        else
        {
            movus_forward(jog_distance);
        }
    }
    else
    {
        if (++approach_counter > lost_tag_max)
        {
            movus_stop();
            set_docking_state("searching");
        }
    }
}

void autodock_controller::final_approach_state_fun()
{
    if (action_state == "turning" || action_state == "jogging")
        return;

    if (in_view && fabs(pose_set.distance - finish_distance) > 0.1)
    {
        set_docking_state("approach");
        return;
    }

    if (M_PI - fabs(tag_yaw) > pose_set.theta_bounds)
    {
        movus_turn((M_PI - fabs(tag_yaw)) * sign(-tag_yaw));
    }
    else
    {
        movus_stop();
        set_docking_state("docked");
        RCLCPP_INFO(get_logger(), "Docking Complete!");
    }
}

void autodock_controller::movus_forward(double distance)
{
    if (action_state.empty())
    {
        set_action_state("jogging");

        double cmd_vel_linear = (distance > 0) ? cmd_vel_linear_rate : -cmd_vel_linear_rate;
        if (fabs(pose_set.distance) < final_approach_distance)
            cmd_vel_linear /= 2;

        cmd_vel_msg.linear.x = cmd_vel_linear;
        robot_point_temp = robot_point;
        temp_distance = distance;
    }
}

void autodock_controller::movus_stop()
{
    set_action_state("");
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;
}

void autodock_controller::movus_turn(double radians)
{
    if (action_state.empty())
    {
        set_action_state("turning");

        double cmd_vel_angular = (radians > 0) ? -cmd_vel_angular_rate : cmd_vel_angular_rate;
        if (fabs(radians) < 0.1)
            cmd_vel_angular /= 2;

        cmd_vel_msg.angular.z = cmd_vel_angular;
        robot_point_temp = robot_point;
        temp_theta = radians;
    }
}

void autodock_controller::action_state_manage()
{
    if (action_state == "jogging")
    {
        if (distance(robot_point_temp, robot_point) >= fabs(temp_distance))
        {
            movus_stop();
            if (docking_state == "approach")
                set_docking_state("centering");
        }
    }
    else if (action_state == "turning")
    {
        if (fabs(robot_point_temp[2] - robot_point[2]) >= fabs(temp_theta))
        {
            movus_stop();
        }
        else if (fabs(pose_set.theta) < pose_set.theta_bounds &&
                 docking_state != "blind" && docking_state != "final_approach")
        {
            movus_stop();
        }
    }

    if (docking_state != "")
        vel_pub->publish(cmd_vel_msg);
}

void autodock_controller::tags_callback()
{
    if (action_state == "count_tag_callbacks")
    {
        tag_callback_counter++;
    }
    else
    {
        tag_callback_counter = 0;
    }

    if (docking_state == "searching" && in_view)
    {
        if (fabs(tag_y / tag_x) >= fabs(tag_x / 2))
        {
            movus_stop();
            set_docking_state("blind");
        }
        else
        {
            movus_stop();
            set_docking_state("centering");
        }
    }
}

void autodock_controller::fid2pos()
{
    double x_trans = tf_bot2dock.transform.translation.x;
    double y_trans = tf_bot2dock.transform.translation.y;
    double theta = atan2(-y_trans, x_trans);
    double r = sqrt(pow(x_trans, 2) + pow(y_trans, 2));
    double theta_bounds;

    if (r > 3.0)
    {
        theta_bounds = approach_angle;
    }
    else
    {
        theta_bounds = r / 30.0;
    }

    // RCLCPP_INFO(get_logger(),"Theta: %3.3f, distance: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds);
    pose_set = {theta - desire_angle * sign(tag_y), r, theta_bounds};
}

void autodock_controller::transform_filter(geometry_msgs::msg::TransformStamped &tf_)
{
    double time = tf_.header.stamp.sec + double(tf_dock2bot.header.stamp.nanosec) * (1e-9);
    if (time == last_time)
    {
        in_view = false;
    }
    else
    {
        in_view = true;
    }
    last_time = time;
}

void autodock_controller::receive_tf()
{
    try
    {
        // tf_odom = buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        // tf_odom = buffer_->lookupTransform("map", "chassis", tf2::TimePointZero);
        tf_odom = buffer_->lookupTransform("camera_optical_link", "apriltag", tf2::TimePointZero);
        odom_x = tf_odom.transform.translation.x;
        odom_y = tf_odom.transform.translation.y;
        odom_yaw = tf2::getYaw(tf_odom.transform.rotation);
        robot_point = {odom_x, odom_y, odom_yaw};

        tf_dock2bot = buffer_->lookupTransform(tag_frame, "camera_optical_link", tf2::TimePointZero);
        tf_bot2dock = buffer_->lookupTransform("camera_optical_link", tag_frame, tf2::TimePointZero);
        // tf_dock2bot = buffer_->lookupTransform("apriltag", "camera_optical_link", tf2::TimePointZero);
        // tf_bot2dock = buffer_->lookupTransform("camera_optical_link", "apriltag", tf2::TimePointZero);
        transform_filter(tf_dock2bot);
        if (!in_view)
        {
            // RCLCPP_WARN(get_logger(),"Tag Detection Lost");
            return;
        }
        tag_x = tf_dock2bot.transform.translation.x;
        tag_y = tf_dock2bot.transform.translation.y;
        tag_yaw = tf2::getYaw(tf_dock2bot.transform.rotation);
        fid2pos();
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
        in_view = false;
    }
}

void autodock_controller::run()
{
    receive_tf();
    tags_callback();
    docking_state_manage();
    action_state_manage();
    state_publish();
}

// Additional methods like fid2pos(), transform_filter(), receive_tf(), and run() remain the same.

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<automatic_parking::autodock_controller>();
    rclcpp::Rate rate(30.0);

    controller_node->set_docking_state("");

    while (rclcpp::ok())
    {
        controller_node->run();
        rclcpp::spin_some(controller_node);
        rate.sleep();
    }

    return 0;
}