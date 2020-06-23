#pragma once

#include <cras_cpp_common/diagnostics/ImprovedUpdater.h>
#include <cras_cpp_common/diagnostics/SlowTopicDiagUpdater.h>
#include <cras_cpp_common/param_utils.h>
#include <cras_cpp_common/type_utils.hpp>

namespace cras
{
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
    if (param->hasParam("rate/desired"))
    {
      this->desiredRate = param->getParam("rate/desired", this->desiredRate);
      this->minRate = param->getParam("rate/min", this->desiredRate);
      this->maxRate = param->getParam("rate/max", this->desiredRate);
    }
    else if (param->hasParam("rate/min") && param->hasParam("rate/max"))
    {
      this->minRate = param->getParam("rate/min", this->minRate);
      this->maxRate = param->getParam("rate/max", this->maxRate);
      this->desiredRate = param->getParam("rate/desired",
        ros::Rate((this->maxRate.expectedCycleTime() - this->minRate.expectedCycleTime()) * 0.5));
    }
    else if (param->hasParam("rate/min") && !param->hasParam("rate/max"))
    {
      this->minRate = param->getParam("rate/min", this->minRate);
      this->desiredRate = param->getParam("rate/desired", this->minRate);
      this->maxRate = param->getParam("rate/max", this->minRate);
    }
    else if (!param->hasParam("rate/min") && param->hasParam("rate/max"))
    {
      this->maxRate = param->getParam("rate/max", this->maxRate);
      this->desiredRate = param->getParam("rate/desired", this->maxRate);
      this->minRate = param->getParam("rate/min", this->maxRate);
    }
    else
    {
      this->desiredRate = param->getParam("rate/desired", this->desiredRate);
      this->minRate = param->getParam("rate/min", this->minRate);
      this->maxRate = param->getParam("rate/max", this->maxRate);
    }

    // recompute the double values
    this->setMinRate(this->minRate);
    this->setMaxRate(this->maxRate);

    this->rateTolerance = param->getParam("rate/tolerance", 0.1);
    this->rateWindowSize = param->getParam("rate/window_size", 5_sz, "updates");
    this->freq = std::make_unique<diagnostic_updater::FrequencyStatusParam>(&this->minRateDbl, &this->maxRateDbl,
                                                                            rateTolerance, rateWindowSize);

    this->minAcceptableDelay = param->getParam("delay/min", ros::Duration(-1));
    this->maxAcceptableDelay = param->getParam("delay/max", ros::Duration(5.0));
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