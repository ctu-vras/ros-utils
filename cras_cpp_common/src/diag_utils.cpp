#include <cras_cpp_common/diag_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>

namespace cras
{
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

const std::string &TopicDiagnostic::getName() const {
  return this->diag->getName();
}

void TopicDiagnostic::addTo(diagnostic_updater::Updater &updater) const {
  updater.add(*this->diag);
}
}
