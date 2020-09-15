#ifndef HAS_SLOW_TOPIC_UPDATER
#include <sstream>
#define private protected
#endif
#include <diagnostic_updater/update_functions.h>
#ifndef HAS_SLOW_TOPIC_UPDATER
#undef private
#endif

#include <cras_cpp_common/diagnostics/SlowTopicDiagUpdater.h>

namespace diagnostic_updater
{

#ifndef HAS_SLOW_TOPIC_UPDATER
SlowTimeStampStatus::SlowTimeStampStatus(const TimeStampStatusParam& params)
    : TimeStampStatus(params)
{
    this->someDataSeen = false;
}

void SlowTimeStampStatus::run(DiagnosticStatusWrapper& stat)
{
    double min_delta;
    double max_delta;
    bool zero_seen;
    {
        boost::mutex::scoped_lock lock(this->lock_);
        min_delta = this->min_delta_;
        max_delta = this->max_delta_;
        zero_seen = this->zero_seen_;
        if (this->deltas_valid_)
            this->someDataSeen = true;
    }

    this->deltas_valid_ = true;
    TimeStampStatus::run(stat);

    if (!this->someDataSeen)
        stat.summary(1, "No data seen yet.");

    {
        boost::mutex::scoped_lock lock(this->lock_);
        this->min_delta_ = min_delta;
        this->max_delta_ = max_delta;
        this->zero_seen_ = zero_seen;
    }
}

void SlowTimeStampStatus::tick(const ros::Time& time)
{
    {
        boost::mutex::scoped_lock lock(this->lock_);
        if (time != ros::Time(0) && !this->deltas_valid_)
           this->zero_seen_ = false;
    }
    TimeStampStatus::tick(time);
}
#endif

SlowTopicDiagnostic::SlowTopicDiagnostic(const std::string& name, Updater& diag,
    const FrequencyStatusParam& freq, const TimeStampStatusParam& stamp)
    : CompositeDiagnosticTask(name), freq(freq), stamp(stamp)
{
    addTask(&this->freq);
    addTask(&this->stamp);

    diag.add(*this);
}

void SlowTopicDiagnostic::tick(const ros::Time& stamp)
{
    this->stamp.tick(stamp);
    this->freq.tick();
}

}