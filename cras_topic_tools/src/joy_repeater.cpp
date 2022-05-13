#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace cras_cpp_common
{
class JoyRepeater
{
public:
    JoyRepeater(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh),
            max_age_(std::numeric_limits<float>::quiet_NaN()),
            source_rate_(std::numeric_limits<float>::quiet_NaN()),
            rate_(std::numeric_limits<float>::quiet_NaN()),
            republish_(true),
            num_published_(0)
    {
        pnh.param("max_age", max_age_, max_age_);
        ROS_INFO("Maximum age: %.3g s.", max_age_);
        pnh.param("source_rate", source_rate_, source_rate_);
        ROS_INFO("Source rate: %.3g s.", source_rate_);
        if (!std::isnan(source_rate_) && (!std::isfinite(source_rate_) || source_rate_ <= 0.0))
        {
            throw std::invalid_argument("Source rate must be positive or NaN.");
        }
        pnh.param("rate", rate_, rate_);
        ROS_INFO("Rate: %.3g s.", rate_);
        if (std::isnan(rate_) || !std::isfinite(rate_) || rate_ <= 0.0)
        {
            throw std::invalid_argument("Rate must be positive.");
        }
        pnh.param("republish", republish_, republish_);
        ROS_INFO("Republish: %i.", republish_);
        pub_ = nh_.advertise<sensor_msgs::Joy>("out", 5, false);
        sub_ = nh_.subscribe("in", 5, &JoyRepeater::messageReceived, this);
        timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &JoyRepeater::everyPeriod, this);
    }

    void publish()
    {
        float age = static_cast<float>((ros::Time::now() - msg_->header.stamp).toSec());
        if (age > max_age_)
        {
            ROS_WARN_THROTTLE(5.0, "Message too old (%.3g s > %.3g s) will not be republished.", age, max_age_);
            return;
        }
        if (!std::isnan(source_rate_) && num_published_ > (rate_ / source_rate_))
        {
            ROS_DEBUG("Message already republished %i times.", num_published_);
            return;
        }
        sensor_msgs::Joy msg = *msg_;
        msg.header.stamp = ros::Time::now();
        pub_.publish(msg);
        ++num_published_;
    }

    void messageReceived(const sensor_msgs::Joy::ConstPtr& msg)
    {
        float age = static_cast<float>((ros::Time::now() - msg->header.stamp).toSec());
        if (age > max_age_)
        {
            ROS_INFO_THROTTLE(5.0, "Received message too old (%.3g s > %.3g s), it will be discarded.", age, max_age_);
            return;
        }
        if (msg_ != nullptr && msg_->header.stamp > msg->header.stamp)
        {
            float older = static_cast<float>((msg_->header.stamp - msg->header.stamp).toSec());
            ROS_INFO_THROTTLE(5.0, "Received message %.3g s older than current message, it will be discarded.", older);
            return;
        }
        std::lock_guard<std::mutex> lock(msg_mutex_);
        msg_ = msg;
        num_published_ = 0;
        if (republish_)
        {
            publish();
        }
    }

    void everyPeriod(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        if (msg_ == nullptr)
        {
            ROS_WARN_THROTTLE(5.0, "Message not yet received.");
            return;
        }
        publish();
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    float max_age_;
    float source_rate_;
    float rate_;
    bool republish_;
    int num_published_;
    std::mutex msg_mutex_;
    sensor_msgs::Joy::ConstPtr msg_;
    ros::Timer timer_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_repeater");
    ros::NodeHandle nh, pnh("~");
    cras_cpp_common::JoyRepeater node(nh, pnh);
    ros::spin();
    return 0;
}
