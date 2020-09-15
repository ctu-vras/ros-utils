#pragma once

#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/diagnostic_updater.h>

/**
 * \brief This is a workaround for bug https://github.com/ros/diagnostics/issues/45 - topics
 * updated less than once a second are wrongly reported as slow even though they actually reach
 * their expected frequency.
 */

namespace diagnostic_updater {

#ifndef HAS_SLOW_TOPIC_UPDATER
class SlowTimeStampStatus : public TimeStampStatus {
  public: explicit SlowTimeStampStatus(const TimeStampStatusParam &params);
  public: ~SlowTimeStampStatus() override = default;
  public: void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;
  public: virtual void tick(const ros::Time& time);
  protected: bool someDataSeen;
};
#endif

class SlowTopicDiagnostic : public CompositeDiagnosticTask {
  public: SlowTopicDiagnostic(const std::string& name, diagnostic_updater::Updater &diag,
                              const diagnostic_updater::FrequencyStatusParam &freq,
                              const diagnostic_updater::TimeStampStatusParam &stamp);
  public: ~SlowTopicDiagnostic() override = default;
  public: virtual void tick(const ros::Time &stamp);

  protected: diagnostic_updater::FrequencyStatus freq;
  protected: diagnostic_updater::SlowTimeStampStatus stamp;
};

template<class T, class D>
class DiagnosedPublisherBase
{
public:
  DiagnosedPublisherBase(const ros::Publisher &pub, D* diag) : publisher_(pub), diag_(diag)
  {
  }

  virtual ~DiagnosedPublisherBase() = default;

  virtual void publish(const boost::shared_ptr<T> &message)
  {
    this->diag_->tick(message->header.stamp);
    this->publisher_.publish(message);
  }

  virtual void publish(const T &message)
  {
    this->diag_->tick(message.header.stamp);
    this->publisher_.publish(message);
  }

  ros::Publisher getPublisher() const
  {
    return this->publisher_;
  }

  void setPublisher(const ros::Publisher& pub)
  {
    this->publisher_ = pub;
  }

private:
  ros::Publisher publisher_;
  D* diag_;
};

template<class T>
class SlowDiagnosedPublisher : public SlowTopicDiagnostic, public DiagnosedPublisherBase<T, SlowTopicDiagnostic>
{
public:
  SlowDiagnosedPublisher(const ros::Publisher &pub, diagnostic_updater::Updater &diag,
                         const diagnostic_updater::FrequencyStatusParam &freq,
                         const diagnostic_updater::TimeStampStatusParam &stamp) :
    SlowTopicDiagnostic(pub.getTopic(), diag, freq, stamp),
    DiagnosedPublisherBase<T, SlowTopicDiagnostic>(pub, this)
  {
  }
  virtual ~SlowDiagnosedPublisher() = default;
};

}