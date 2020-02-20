#include <mutex>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace cras_cpp_common
{
class Repeater
{
public:
    Repeater(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh),
            rate_(std::numeric_limits<float>::quiet_NaN()),
            republish_(true),
            advertised_(false)
    {
        pnh.param("rate", rate_, rate_);
        pnh.param("republish", republish_, republish_);
        if (std::isnan(rate_) || !std::isfinite(rate_) || rate_ <= 0.0)
        {
            throw std::invalid_argument("Rate must be positive.");
        }
        sub_ = nh_.subscribe("in", 5, &Repeater::messageReceived, this);
        timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &Repeater::everyPeriod, this);
    }

    void messageReceived(const topic_tools::ShapeShifter::ConstPtr& msg)
    {
        if (!advertised_)
        {
            pub_ = msg->advertise(nh_, "out", 5, false);
            advertised_ = true;
        }
        if (republish_)
        {
            pub_.publish(msg);
        }
        std::lock_guard<std::mutex> lock(msg_mutex_);
        msg_ = msg;
    }

    void everyPeriod(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        pub_.publish(msg_);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    float rate_;
    bool republish_;
    bool advertised_;
    std::mutex msg_mutex_;
    topic_tools::ShapeShifter::ConstPtr msg_;
    ros::Timer timer_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "repeater");
    ros::NodeHandle nh, pnh("~");
    cras_cpp_common::Repeater node(nh, pnh);
    ros::spin();
    return 0;
}
