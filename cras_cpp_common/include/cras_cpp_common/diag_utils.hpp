#pragma once

#include <cras_cpp_common/diagnostics/ImprovedUpdater.h>
#include <cras_cpp_common/diagnostics/SlowTopicDiagUpdater.h>
#include <cras_cpp_common/param_utils.h>
#include <cras_cpp_common/type_utils.hpp>

namespace cras
{
  class TopicDiagnostic : public diagnostic_updater::SlowTopicDiagnostic, public std::enable_shared_from_this<TopicDiagnostic>
  {
  public:
    TopicDiagnostic(const std::string& name, diagnostic_updater::Updater& diag, cras::BoundParamHelperPtr param);
    virtual ~TopicDiagnostic() = default;

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

    diagnostic_updater::FrequencyStatusParam freq;
    diagnostic_updater::TimeStampStatusParam stamps;
  };

  TopicDiagnostic::TopicDiagnostic(const std::string& name, diagnostic_updater::Updater &diag,
    cras::BoundParamHelperPtr param) :
    minRateDbl(-1.0), maxRateDbl(-1.0), minRate(-1.0), desiredRate(-1.0), maxRate(-1.0),  // rates will be set in the body
    rateTolerance(param->getParam("tolerance", 0.1)),
    rateWindowSize(param->getParam("window_size", 5_sz, "updates")),
    freq(&this->minRateDbl, &this->maxRateDbl, rateTolerance, rateWindowSize),
    minAcceptableDelay(param->getParam("min_delay", ros::Duration(-1))),
    maxAcceptableDelay(param->getParam("max_delay", ros::Duration(5.0))),
    stamps(minAcceptableDelay.toSec(), maxAcceptableDelay.toSec()),
    diagnostic_updater::SlowTopicDiagnostic(name, diag, freq, stamps)
  {
      if (param->hasParam("rate"))
      {
        this->desiredRate = param->getParam("rate", ros::Rate(-1.0));
        this->minRate = param->getParam("min_rate", this->desiredRate);
        this->maxRate = param->getParam("max_rate", this->desiredRate);
      }
      else if (param->hasParam("min_rate") && param->hasParam("max_rate"))
      {
        this->minRate = param->getParam("min_rate", ros::Rate(-1.0));
        this->maxRate = param->getParam("max_rate", ros::Rate(-1.0));
        this->desiredRate = param->getParam("rate", ros::Rate((this->maxRate.expectedCycleTime() - this->minRate.expectedCycleTime()) * 0.5));
      }
      else if (param->hasParam("min_rate") && !param->hasParam("max_rate"))
      {
        this->minRate = param->getParam("min_rate", ros::Rate(-1.0));
        this->desiredRate = param->getParam("rate", this->minRate);
        this->maxRate = param->getParam("max_rate", this->minRate);
      }
      else if (!param->hasParam("min_rate") && param->hasParam("max_rate"))
      {
        this->maxRate = param->getParam("max_rate", ros::Rate(-1.0));
        this->desiredRate = param->getParam("rate", this->maxRate);
        this->minRate = param->getParam("min_rate", this->maxRate);
      }
      else
      {
        this->desiredRate = param->getParam("rate", ros::Rate(-1.0));
        this->minRate = param->getParam("min_rate", ros::Rate(-1.0));
        this->maxRate = param->getParam("max_rate", ros::Rate(-1.0));
      }

      // recompute the double values
      this->setMinRate(this->minRate);
      this->setMaxRate(this->maxRate);
  }

  template <typename T>
  class DiagnosedPublisher : public TopicDiagnostic, public diagnostic_updater::SlowDiagnosedPublisher<T>
  {
  public:
    DiagnosedPublisher(const ros::Publisher &pub, diagnostic_updater::Updater &diag,
                       cras::BoundParamHelperPtr param) :
        TopicDiagnostic(pub.getTopic(), diag, param),
        diagnostic_updater::SlowDiagnosedPublisher<T>(pub, TopicDiagnostic::shared_from_this())
    {
    }
  };
}