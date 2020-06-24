#pragma once

#include <cras_cpp_common/diagnostics/ImprovedUpdater.h>
#include <cras_cpp_common/diagnostics/SlowTopicDiagUpdater.h>
#include <cras_cpp_common/param_utils.h>
#include <cras_cpp_common/type_utils.hpp>

namespace cras
{

/**
 * This class does frequency and delay diagnostics of a published/subscribed topic and allows configuration via
 * parameter server. The configuration is stored in a parameter namespace which you pass with the BoundParamHelper. For
 * this diagnostics to work, you have to call tick() every time you send/receive a message.
 *
 * The configuration parameters (relative to the namespace) are:
 *  - rate/desired (double): The desired rate of messages.
 *  - rate/min (double): Min rate of messages.
 *  - rate/max (double): Max rate of messages.
 *  - rate/tolerance (double): Tolerance of min/max rate (decreases min, increases max, or goes both ways from desired).
 *  - rate/window_size (positive int): Number of tick() calls throughout which the rate should be approximated.
 *  - delay/min (double): Minimum delay of the message (difference between now and message header).
 *  - delay/max (double): Maximum delay of the message (difference between now and message header).
 *
 * The rate parameters are processed according to the following rules:
 *  - If exactly one of (desired|min|max) is set, it is used for all three rates.
 *  - If min and max are set and desired is not set, desired is set to the mean of min and max.
 *  - If no rate parameter is set, a rate of -1 Hz is used for all three rates. This default can be changed in the
 *    extended constructors by passing defaultRate, defaultMinRate and defaultMaxRate.
 *
 * The rate (desired|min|max) parameters can be changed dynamically during runtime. Other parameters are constant.
 */
class TopicDiagnostic
{
public:
  TopicDiagnostic(const std::string& name, diagnostic_updater::Updater& updater, cras::BoundParamHelperPtr param,
                  const ros::Rate& defaultRate, const ros::Rate& defaultMinRate, const ros::Rate& defaultMaxRate);
  TopicDiagnostic(const std::string& name, diagnostic_updater::Updater& updater, cras::BoundParamHelperPtr param,
                  const ros::Rate& defaultRate);
  TopicDiagnostic(const std::string& name, diagnostic_updater::Updater& updater, cras::BoundParamHelperPtr param);

  virtual ~TopicDiagnostic() {};

  void tick(const ros::Time& stamp) const;

  inline const ros::Rate& getMinRate() const { return this->minRate; }
  inline const ros::Rate& getDesiredRate() const { return this->desiredRate; }
  inline const ros::Rate& getMaxRate() const { return this->maxRate; }
  inline const void setMinRate(const ros::Rate& rate) { this->minRate = rate; this->minRateDbl = 1.0 / rate.expectedCycleTime().toSec(); }
  inline const void setDesiredRate(const ros::Rate& rate) { this->desiredRate = rate; }
  inline const void setMaxRate(const ros::Rate& rate) { this->maxRate = rate; this->maxRateDbl = 1.0 / rate.expectedCycleTime().toSec(); }
  inline double getRateTolerance() const { return this->rateTolerance; }
  inline size_t getRateWindowSize() const { return this->rateWindowSize; }
  inline const ros::Duration& getMinAcceptableDelay() const { return this->minAcceptableDelay; }
  inline const ros::Duration& getMaxAcceptableDelay() const { return this->maxAcceptableDelay; }

protected:
  ros::Rate minRate;
  ros::Rate desiredRate;
  ros::Rate maxRate;
  double minRateDbl;
  double maxRateDbl;
  double rateTolerance;
  size_t rateWindowSize;
  ros::Duration minAcceptableDelay;
  ros::Duration maxAcceptableDelay;

  std::unique_ptr<diagnostic_updater::FrequencyStatusParam> freq;
  std::unique_ptr<diagnostic_updater::TimeStampStatusParam> stamps;

  std::unique_ptr<diagnostic_updater::SlowTopicDiagnostic> diag;
};

TopicDiagnostic::TopicDiagnostic(const std::string& name, diagnostic_updater::Updater &diag,
  cras::BoundParamHelperPtr param, const ros::Rate& defaultRate, const ros::Rate& defaultMinRate,
  const ros::Rate& defaultMaxRate) : minRate(defaultMinRate), desiredRate(defaultRate), maxRate(defaultMaxRate)
{
  const auto rateParam = param->paramsInNamespace("rate");
  const auto delayParam = param->paramsInNamespace("delay");

  if (rateParam->hasParam("desired"))
  {
    this->desiredRate = rateParam->getParam("desired", this->desiredRate);
    this->minRate = rateParam->getParam("min", this->desiredRate);
    this->maxRate = rateParam->getParam("max", this->desiredRate);
  }
  else if (rateParam->hasParam("min") && rateParam->hasParam("max"))
  {
    this->minRate = rateParam->getParam("min", this->minRate);
    this->maxRate = rateParam->getParam("max", this->maxRate);
    this->desiredRate = rateParam->getParam("desired",
      ros::Rate((this->maxRate.expectedCycleTime() - this->minRate.expectedCycleTime()) * 0.5));
  }
  else if (rateParam->hasParam("min") && !rateParam->hasParam("max"))
  {
    this->minRate = rateParam->getParam("min", this->minRate);
    this->desiredRate = rateParam->getParam("desired", this->minRate);
    this->maxRate = rateParam->getParam("max", this->minRate);
  }
  else if (!rateParam->hasParam("min") && rateParam->hasParam("max"))
  {
    this->maxRate = rateParam->getParam("max", this->maxRate);
    this->desiredRate = rateParam->getParam("desired", this->maxRate);
    this->minRate = rateParam->getParam("min", this->maxRate);
  }
  else
  {
    this->desiredRate = rateParam->getParam("desired", this->desiredRate);
    this->minRate = rateParam->getParam("min", this->minRate);
    this->maxRate = rateParam->getParam("max", this->maxRate);
  }

  // recompute the double values
  this->setMinRate(this->minRate);
  this->setMaxRate(this->maxRate);

  this->rateTolerance = rateParam->getParam("tolerance", 0.1);
  this->rateWindowSize = rateParam->getParam("window_size", 5_sz, "updates");
  this->freq = std::make_unique<diagnostic_updater::FrequencyStatusParam>(&this->minRateDbl, &this->maxRateDbl,
                                                                          rateTolerance, rateWindowSize);

  this->minAcceptableDelay = delayParam->getParam("min", ros::Duration(-1));
  this->maxAcceptableDelay = delayParam->getParam("max", ros::Duration(5.0));
  this->stamps = std::make_unique<diagnostic_updater::TimeStampStatusParam>(minAcceptableDelay.toSec(),
                                                                            maxAcceptableDelay.toSec());

  this->diag = std::make_unique<diagnostic_updater::SlowTopicDiagnostic>(name, diag, *this->freq, *this->stamps);
}

TopicDiagnostic::TopicDiagnostic(const std::string& name, diagnostic_updater::Updater &diag,
  cras::BoundParamHelperPtr param, const ros::Rate& defaultRate) :
  TopicDiagnostic(name, diag, param, defaultRate, defaultRate, defaultRate)
{
}

TopicDiagnostic::TopicDiagnostic(const std::string& name, diagnostic_updater::Updater &diag,
  cras::BoundParamHelperPtr param) :
  TopicDiagnostic(name, diag, param, ros::Rate(-1.0))
{
}

void TopicDiagnostic::tick(const ros::Time& stamp) const {
  this->diag->tick(stamp);
}

/**
 * This is a convenience class for TopicDiagnostic. If you want to diagnose a publisher, you can use this class and
 * publish the messages using it, and it will automatically call tick() for you every time a message is published.
 * @tparam T Type of the published messages.
 */
template<class T>
class DiagnosedPublisher : public TopicDiagnostic, public diagnostic_updater::DiagnosedPublisherBase<T, TopicDiagnostic>
{
public:
  DiagnosedPublisher(const ros::Publisher &pub, diagnostic_updater::Updater &diag,
                     cras::BoundParamHelperPtr param, const ros::Rate& defaultRate, const ros::Rate& defaultMinRate,
                     const ros::Rate& defaultMaxRate) :
      TopicDiagnostic(pub.getTopic(), diag, param, defaultRate, defaultMinRate, defaultMaxRate),
      diagnostic_updater::DiagnosedPublisherBase<T, TopicDiagnostic>(pub, this)
  {
  }

  DiagnosedPublisher(const ros::Publisher &pub, diagnostic_updater::Updater &diag,
                     cras::BoundParamHelperPtr param, const ros::Rate& defaultRate) :
      TopicDiagnostic(pub.getTopic(), diag, param, defaultRate),
      diagnostic_updater::DiagnosedPublisherBase<T, TopicDiagnostic>(pub, this)
  {
  }

  DiagnosedPublisher(const ros::Publisher &pub, diagnostic_updater::Updater &diag,
                     cras::BoundParamHelperPtr param) :
      TopicDiagnostic(pub.getTopic(), diag, param),
      diagnostic_updater::DiagnosedPublisherBase<T, TopicDiagnostic>(pub, this)
  {
  }

  virtual ~DiagnosedPublisher() = default;
};

}