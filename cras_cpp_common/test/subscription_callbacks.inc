/**
 * \file
 * \brief This file contains common definitions used for testing topic diagnostic utils.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <boost/function.hpp>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/message_event.h>
#include <std_msgs/Header.h>

class CbTest
{
public:
  void cbConstPtr(const std_msgs::HeaderConstPtr& m) {this->cbConstPtrFrame = m->frame_id;}
  void cbPtr(const std_msgs::HeaderPtr& m) {this->cbPtrFrame = m->frame_id;}
  void cbConstRef(const std_msgs::Header& m) {this->cbConstRefFrame = m.frame_id;}
  void cbValue(std_msgs::Header m) {this->cbValueFrame = m.frame_id;}  // NOLINT
  void cbConstEvent(const ros::MessageEvent<std_msgs::Header const>& m) {
    this->cbConstEventFrame = m.getConstMessage()->frame_id;}
  void cbEvent(const ros::MessageEvent<std_msgs::Header>& m) { this->cbEventFrame = m.getConstMessage()->frame_id;}
  void constCbConstPtr(const std_msgs::HeaderConstPtr& m) const {this->constCbConstPtrFrame = m->frame_id;}
  void constCbPtr(const std_msgs::HeaderPtr& m) const {this->constCbPtrFrame = m->frame_id;}
  void constCbConstRef(const std_msgs::Header& m) const {this->constCbConstRefFrame = m.frame_id;}
  void constCbValue(std_msgs::Header m) const {this->constCbValueFrame = m.frame_id;}  // NOLINT
  void constCbConstEvent(const ros::MessageEvent<std_msgs::Header const>& m) const {
    this->constCbConstEventFrame = m.getConstMessage()->frame_id;}
  void constCbEvent(const ros::MessageEvent<std_msgs::Header>& m) const {
    this->constCbEventFrame = m.getConstMessage()->frame_id;}

  void addStringsTo(std::vector<std::string*>& strs)
  {
    strs.push_back(&this->cbConstPtrFrame);
    strs.push_back(&this->cbPtrFrame);
    strs.push_back(&this->cbConstRefFrame);
    strs.push_back(&this->cbValueFrame);
    strs.push_back(&this->cbConstEventFrame);
    strs.push_back(&this->cbEventFrame);
    strs.push_back(&this->constCbConstPtrFrame);
    strs.push_back(&this->constCbPtrFrame);
    strs.push_back(&this->constCbConstRefFrame);
    strs.push_back(&this->constCbValueFrame);
    strs.push_back(&this->constCbConstEventFrame);
    strs.push_back(&this->constCbEventFrame);
  }

  std::string cbConstPtrFrame {};
  std::string cbPtrFrame {};
  std::string cbConstRefFrame {};
  std::string cbValueFrame {};
  std::string cbConstEventFrame {};
  std::string cbEventFrame {};
  mutable std::string constCbConstPtrFrame {};
  mutable std::string constCbPtrFrame {};
  mutable std::string constCbConstRefFrame {};
  mutable std::string constCbValueFrame {};
  mutable std::string constCbConstEventFrame {};
  mutable std::string constCbEventFrame {};
};

class CbTestHeader
{
public:
  void cbConstPtr(const diagnostic_msgs::DiagnosticArrayConstPtr& m) {this->cbConstPtrFrame = m->header.frame_id;}
  void cbPtr(const diagnostic_msgs::DiagnosticArrayPtr& m) {this->cbPtrFrame = m->header.frame_id;}
  void cbConstRef(const diagnostic_msgs::DiagnosticArray& m) {this->cbConstRefFrame = m.header.frame_id;}
  void cbValue(diagnostic_msgs::DiagnosticArray m) {this->cbValueFrame = m.header.frame_id;}  // NOLINT
  void cbConstEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
    this->cbConstEventFrame = m.getConstMessage()->header.frame_id;}
  void cbEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
    this->cbEventFrame = m.getConstMessage()->header.frame_id;}
  void constCbConstPtr(const diagnostic_msgs::DiagnosticArrayConstPtr& m) const {
    this->constCbConstPtrFrame = m->header.frame_id;}
  void constCbPtr(const diagnostic_msgs::DiagnosticArrayPtr& m) const {this->constCbPtrFrame = m->header.frame_id;}
  void constCbConstRef(const diagnostic_msgs::DiagnosticArray& m) const {this->constCbConstRefFrame = m.header.frame_id;}
  void constCbValue(diagnostic_msgs::DiagnosticArray m) const {this->constCbValueFrame = m.header.frame_id;}  // NOLINT
  void constCbConstEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) const {
    this->constCbConstEventFrame = m.getConstMessage()->header.frame_id;}
  void constCbEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) const {
    this->constCbEventFrame = m.getConstMessage()->header.frame_id;}

  void addStringsTo(std::vector<std::string*>& strs)
  {
    strs.push_back(&this->cbConstPtrFrame);
    strs.push_back(&this->cbPtrFrame);
    strs.push_back(&this->cbConstRefFrame);
    strs.push_back(&this->cbValueFrame);
    strs.push_back(&this->cbConstEventFrame);
    strs.push_back(&this->cbEventFrame);
    strs.push_back(&this->constCbConstPtrFrame);
    strs.push_back(&this->constCbPtrFrame);
    strs.push_back(&this->constCbConstRefFrame);
    strs.push_back(&this->constCbValueFrame);
    strs.push_back(&this->constCbConstEventFrame);
    strs.push_back(&this->constCbEventFrame);
  }

  std::string cbConstPtrFrame {};
  std::string cbPtrFrame {};
  std::string cbConstRefFrame {};
  std::string cbValueFrame {};
  std::string cbConstEventFrame {};
  std::string cbEventFrame {};
  mutable std::string constCbConstPtrFrame {};
  mutable std::string constCbPtrFrame {};
  mutable std::string constCbConstRefFrame {};
  mutable std::string constCbValueFrame {};
  mutable std::string constCbConstEventFrame {};
  mutable std::string constCbEventFrame {};
};

std::string cbConstPtrFrame {};
std::string cbPtrFrame {};
std::string cbConstRefFrame {};
std::string cbValueFrame {};
std::string cbConstEventFrame {};
std::string cbEventFrame {};

void cbConstPtr(const std_msgs::HeaderConstPtr& m) {cbConstPtrFrame = m->frame_id;}
void cbPtr(const std_msgs::HeaderPtr& m) {cbPtrFrame = m->frame_id;}
void cbConstRef(const std_msgs::Header& m) {cbConstRefFrame = m.frame_id;}
void cbValue(std_msgs::Header m) {cbValueFrame = m.frame_id;}  // NOLINT
void cbConstEvent(const ros::MessageEvent<std_msgs::Header const>& m) {
  cbConstEventFrame = m.getConstMessage()->frame_id;}
void cbEvent(const ros::MessageEvent<std_msgs::Header>& m) {cbEventFrame = m.getConstMessage()->frame_id;}

void cbHeaderConstPtr(const diagnostic_msgs::DiagnosticArrayConstPtr& m) {cbConstPtrFrame = m->header.frame_id;}
void cbHeaderPtr(const diagnostic_msgs::DiagnosticArrayPtr& m) {cbPtrFrame = m->header.frame_id;}
void cbHeaderConstRef(const diagnostic_msgs::DiagnosticArray& m) {cbConstRefFrame = m.header.frame_id;}
void cbHeaderValue(diagnostic_msgs::DiagnosticArray m) {cbValueFrame = m.header.frame_id;}  // NOLINT
void cbHeaderConstEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
  cbConstEventFrame = m.getConstMessage()->header.frame_id;}
void cbHeaderEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
  cbEventFrame = m.getConstMessage()->header.frame_id;}

std::string optionsCbConstPtrFrame {};
std::string optionsCbPtrFrame {};
std::string optionsCbConstRefFrame {};
std::string optionsCbValueFrame {};
std::string optionsCbConstEventFrame {};
std::string optionsCbEventFrame {};

void optionsCbConstPtr(const std_msgs::HeaderConstPtr& m) {optionsCbConstPtrFrame = m->frame_id;}
void optionsCbPtr(const std_msgs::HeaderPtr& m) {optionsCbPtrFrame = m->frame_id;}
void optionsCbConstRef(const std_msgs::Header& m) {optionsCbConstRefFrame = m.frame_id;}
void optionsCbValue(std_msgs::Header m) {optionsCbValueFrame = m.frame_id;}  // NOLINT
void optionsCbConstEvent(const ros::MessageEvent<std_msgs::Header const>& m) {
  optionsCbConstEventFrame = m.getConstMessage()->frame_id;}
void optionsCbEvent(const ros::MessageEvent<std_msgs::Header>& m) {optionsCbEventFrame = m.getConstMessage()->frame_id;}

void optionsHeaderCbConstPtr(const diagnostic_msgs::DiagnosticArrayConstPtr& m) {
  optionsCbConstPtrFrame = m->header.frame_id;}
void optionsHeaderCbPtr(const diagnostic_msgs::DiagnosticArrayPtr& m) {optionsCbPtrFrame = m->header.frame_id;}
void optionsHeaderCbConstRef(const diagnostic_msgs::DiagnosticArray& m) {optionsCbConstRefFrame = m.header.frame_id;}
void optionsHeaderCbValue(diagnostic_msgs::DiagnosticArray m) {optionsCbValueFrame = m.header.frame_id;}  // NOLINT
void optionsHeaderCbConstEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
  optionsCbConstEventFrame = m.getConstMessage()->header.frame_id;}
void optionsHeaderCbEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
  optionsCbEventFrame = m.getConstMessage()->header.frame_id;}

std::string lambdaCbConstPtrFrame {};
std::string lambdaCbPtrFrame {};
std::string lambdaCbConstRefFrame {};
std::string lambdaCbValueFrame {};
std::string lambdaCbConstEventFrame {};
std::string lambdaCbEventFrame {};

auto lambdaCbConstPtr = [](const std_msgs::HeaderConstPtr& m){lambdaCbConstPtrFrame = m->frame_id;};
auto lambdaCbPtr = [](const std_msgs::HeaderPtr& m){lambdaCbPtrFrame = m->frame_id;};
auto lambdaCbConstRef = [](const std_msgs::Header& m){lambdaCbConstRefFrame = m.frame_id;};
auto lambdaCbValue = [](std_msgs::Header m){lambdaCbValueFrame = m.frame_id;};  // NOLINT
auto lambdaCbConstEvent = [](const ros::MessageEvent<std_msgs::Header const>& m) {
  lambdaCbConstEventFrame = m.getConstMessage()->frame_id;};
auto lambdaCbEvent = [](const ros::MessageEvent<std_msgs::Header>& m) {
  lambdaCbEventFrame = m.getConstMessage()->frame_id;};

auto lambdaHeaderCbConstPtr = [](const diagnostic_msgs::DiagnosticArrayConstPtr& m){
  lambdaCbConstPtrFrame = m->header.frame_id;};
auto lambdaHeaderCbPtr = [](const diagnostic_msgs::DiagnosticArrayPtr& m){lambdaCbPtrFrame = m->header.frame_id;};
auto lambdaHeaderCbConstRef = [](const diagnostic_msgs::DiagnosticArray& m){lambdaCbConstRefFrame = m.header.frame_id;};
auto lambdaHeaderCbValue = [](diagnostic_msgs::DiagnosticArray m){lambdaCbValueFrame = m.header.frame_id;};  // NOLINT
auto lambdaHeaderCbConstEvent = [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
  lambdaCbConstEventFrame = m.getConstMessage()->header.frame_id;};
auto lambdaHeaderCbEvent = [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
  lambdaCbEventFrame = m.getConstMessage()->header.frame_id;};

std::string lambdaHintsConstPtrFrame {};
std::string lambdaHintsPtrFrame {};
std::string lambdaHintsConstRefFrame {};
std::string lambdaHintsValueFrame {};
std::string lambdaHintsConstEventFrame {};
std::string lambdaHintsEventFrame {};

auto lambdaHintsConstPtr = [](const std_msgs::HeaderConstPtr& m){lambdaHintsConstPtrFrame = m->frame_id;};
auto lambdaHintsPtr = [](const std_msgs::HeaderPtr& m){lambdaHintsPtrFrame = m->frame_id;};
auto lambdaHintsConstRef = [](const std_msgs::Header& m){lambdaHintsConstRefFrame = m.frame_id;};
auto lambdaHintsValue = [](std_msgs::Header m){lambdaHintsValueFrame = m.frame_id;};  // NOLINT
auto lambdaHintsConstEvent = [](const ros::MessageEvent<std_msgs::Header const>& m) {
  lambdaHintsConstEventFrame = m.getConstMessage()->frame_id;};
auto lambdaHintsEvent = [](const ros::MessageEvent<std_msgs::Header>& m) {
  lambdaHintsEventFrame = m.getConstMessage()->frame_id;};

auto lambdaHeaderHintsConstPtr = [](const diagnostic_msgs::DiagnosticArrayConstPtr& m){
  lambdaHintsConstPtrFrame = m->header.frame_id;};
auto lambdaHeaderHintsPtr = [](const diagnostic_msgs::DiagnosticArrayPtr& m){lambdaHintsPtrFrame = m->header.frame_id;};
auto lambdaHeaderHintsConstRef = [](const diagnostic_msgs::DiagnosticArray& m){
  lambdaHintsConstRefFrame = m.header.frame_id;};
auto lambdaHeaderHintsValue = [](diagnostic_msgs::DiagnosticArray m){  //NOLINT
  lambdaHintsValueFrame = m.header.frame_id;};
auto lambdaHeaderHintsConstEvent = [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
  lambdaHintsConstEventFrame = m.getConstMessage()->header.frame_id;};
auto lambdaHeaderHintsEvent = [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
  lambdaHintsEventFrame = m.getConstMessage()->header.frame_id;};

std::string boostLambdaConstPtrFrame {};
std::string boostLambdaPtrFrame {};
std::string boostLambdaConstRefFrame {};
std::string boostLambdaValueFrame {};
std::string boostLambdaConstEventFrame {};
std::string boostLambdaEventFrame {};

boost::function<void(const std_msgs::HeaderConstPtr&)> boostLambdaConstPtr =  // NOLINT
  [](const std_msgs::HeaderConstPtr& m){boostLambdaConstPtrFrame = m->frame_id;};
boost::function<void(const std_msgs::HeaderPtr&)> boostLambdaPtr =  // NOLINT
  [](const std_msgs::HeaderPtr& m){boostLambdaPtrFrame = m->frame_id;};
boost::function<void(const std_msgs::Header&)> boostLambdaConstRef =  // NOLINT
  [](const std_msgs::Header& m){boostLambdaConstRefFrame = m.frame_id;};
boost::function<void(std_msgs::Header)> boostLambdaValue =  // NOLINT
  [](std_msgs::Header m){boostLambdaValueFrame = m.frame_id;};  // NOLINT
boost::function<void(const ros::MessageEvent<std_msgs::Header const>&)> boostLambdaConstEvent =  // NOLINT
  [](const ros::MessageEvent<std_msgs::Header const>& m) {boostLambdaConstEventFrame = m.getConstMessage()->frame_id;};
boost::function<void(const ros::MessageEvent<std_msgs::Header>&)> boostLambdaEvent =  // NOLINT
  [](const ros::MessageEvent<std_msgs::Header>& m) {boostLambdaEventFrame = m.getConstMessage()->frame_id;};

boost::function<void(const diagnostic_msgs::DiagnosticArrayConstPtr&)> boostHeaderLambdaConstPtr =  // NOLINT
  [](const diagnostic_msgs::DiagnosticArrayConstPtr& m){boostLambdaConstPtrFrame = m->header.frame_id;};
boost::function<void(const diagnostic_msgs::DiagnosticArrayPtr&)> boostHeaderLambdaPtr =  // NOLINT
  [](const diagnostic_msgs::DiagnosticArrayPtr& m){boostLambdaPtrFrame = m->header.frame_id;};
boost::function<void(const diagnostic_msgs::DiagnosticArray&)> boostHeaderLambdaConstRef =  // NOLINT
  [](const diagnostic_msgs::DiagnosticArray& m){boostLambdaConstRefFrame = m.header.frame_id;};
boost::function<void(diagnostic_msgs::DiagnosticArray)> boostHeaderLambdaValue =  // NOLINT
  [](diagnostic_msgs::DiagnosticArray m){boostLambdaValueFrame = m.header.frame_id;};  // NOLINT
boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&)>
  boostHeaderLambdaConstEvent =  // NOLINT
  [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
    boostLambdaConstEventFrame = m.getConstMessage()->header.frame_id;};
boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&)> boostHeaderLambdaEvent =  // NOLINT
  [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
    boostLambdaEventFrame = m.getConstMessage()->header.frame_id;};

std::string noCaptureLambdaConstPtrFrame {};
std::string noCaptureLambdaPtrFrame {};
std::string noCaptureLambdaConstRefFrame {};
std::string noCaptureLambdaValueFrame {};
std::string noCaptureLambdaConstEventFrame {};
std::string noCaptureLambdaEventFrame {};
std::string valueCaptureLambdaConstPtrFrame {};
std::string valueCaptureLambdaPtrFrame {};
std::string valueCaptureLambdaConstRefFrame {};
std::string valueCaptureLambdaValueFrame {};
std::string valueCaptureLambdaConstEventFrame {};
std::string valueCaptureLambdaEventFrame {};

#define CRAS_TEST_SINGLE_ARG(...) __VA_ARGS__

#define CRAS_TEST_SUBSCRIBE_CALLBACKS(SUB, ARGS_PRE, ARGS_SUB, HEADER, FRAME_REF, FRAME_PTR, FRAME_EVENT_PTR) \
  TestClass obj1, obj2, obj3;\
  auto sharedObj = boost::make_shared<TestClass>();\
  \
  std::string refCaptureLambdaConstPtrFrame {};\
  std::string refCaptureLambdaPtrFrame {};\
  std::string refCaptureLambdaConstRefFrame {};\
  std::string refCaptureLambdaValueFrame {};\
  std::string refCaptureLambdaConstEventFrame {};\
  std::string refCaptureLambdaEventFrame {};\
  \
  std::vector<std::string*> strs = { \
    &cbConstPtrFrame, &cbPtrFrame, &cbConstRefFrame, &cbValueFrame, &cbConstEventFrame, &cbEventFrame,   /* 5 */ \
    &optionsCbConstPtrFrame, &optionsCbPtrFrame, &optionsCbConstRefFrame, &optionsCbValueFrame,\
    &optionsCbConstEventFrame, &optionsCbEventFrame,                                                     /* 11 */ \
    &lambdaCbConstPtrFrame, &lambdaCbPtrFrame, &lambdaCbConstRefFrame, &lambdaCbValueFrame,\
    &lambdaCbConstEventFrame, &lambdaCbEventFrame,                                                       /* 17 */ \
    &lambdaHintsConstPtrFrame, &lambdaHintsPtrFrame, &lambdaHintsConstRefFrame, &lambdaHintsValueFrame,\
    &lambdaHintsConstEventFrame, &lambdaHintsEventFrame,                                                 /* 23 */ \
    &boostLambdaConstPtrFrame, &boostLambdaPtrFrame, &boostLambdaConstRefFrame, &boostLambdaValueFrame,\
    &boostLambdaConstEventFrame, &boostLambdaEventFrame,                                                 /* 29 */ \
    &noCaptureLambdaConstPtrFrame, &noCaptureLambdaPtrFrame, &noCaptureLambdaConstRefFrame,\
    &noCaptureLambdaValueFrame, &noCaptureLambdaConstEventFrame, &noCaptureLambdaEventFrame,             /* 35 */ \
    &valueCaptureLambdaConstPtrFrame, &valueCaptureLambdaPtrFrame, &valueCaptureLambdaConstRefFrame,\
    &valueCaptureLambdaValueFrame, &valueCaptureLambdaConstEventFrame, &valueCaptureLambdaEventFrame,    /* 41 */ \
    &refCaptureLambdaConstPtrFrame, &refCaptureLambdaPtrFrame, &refCaptureLambdaConstRefFrame,\
    &refCaptureLambdaValueFrame, &refCaptureLambdaConstEventFrame, &refCaptureLambdaEventFrame,          /* 47 */ \
  };\
  obj1.addStringsTo(strs);                                                                               /* 59 */ \
  obj2.addStringsTo(strs);                                                                               /* 71 */ \
  obj3.addStringsTo(strs);                                                                               /* 83 */ \
  sharedObj->addStringsTo(strs);                                                                         /* 95 */ \
  \
  for (const auto str : strs)\
    *str = "";\
  using MsgPtr = typename Msg::Ptr;\
  using MsgConstPtr = typename Msg::ConstPtr;\
  \
  auto subCbConstPtr = SUB(ARGS_PRE ARGS_SUB &cb##HEADER##ConstPtr);\
  auto subCbPtr = SUB(ARGS_PRE ARGS_SUB &cb##HEADER##Ptr);\
  auto subCbConstRef = SUB(ARGS_PRE ARGS_SUB &cb##HEADER##ConstRef);\
  auto subCbValue = SUB(ARGS_PRE ARGS_SUB &cb##HEADER##Value);\
  auto subCbConstEvent = SUB(ARGS_PRE ARGS_SUB &cb##HEADER##ConstEvent);\
  auto subCbEvent = SUB(ARGS_PRE ARGS_SUB &cb##HEADER##Event);\
  \
  auto subClassConstPtr = SUB(ARGS_PRE ARGS_SUB &TestClass::cbConstPtr, &obj1);\
  auto subClassPtr = SUB(ARGS_PRE ARGS_SUB &TestClass::cbPtr, &obj1);\
  auto subClassConstRef = SUB(ARGS_PRE ARGS_SUB &TestClass::cbConstRef, &obj1);\
  auto subClassValue = SUB(ARGS_PRE ARGS_SUB &TestClass::cbValue, &obj1);\
  auto subClassConstEvent = SUB(ARGS_PRE ARGS_SUB &TestClass::cbConstEvent, &obj1);\
  auto subClassEvent = SUB(ARGS_PRE ARGS_SUB &TestClass::cbEvent, &obj1);\
  \
  auto subConstClassConstPtr = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbConstPtr, &obj1);\
  auto subConstClassPtr = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbPtr, &obj1);\
  auto subConstClassConstRef = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbConstRef, &obj1);\
  auto subConstClassValue = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbValue, &obj1);\
  auto subConstClassConstEvent = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbConstEvent, &obj1);\
  auto subConstClassEvent = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbEvent, &obj1);\
  \
  auto subSharedClassConstPtr = SUB(ARGS_PRE ARGS_SUB &TestClass::cbConstPtr, sharedObj);\
  auto subSharedClassPtr = SUB(ARGS_PRE ARGS_SUB &TestClass::cbPtr, sharedObj);\
  auto subSharedClassConstRef = SUB(ARGS_PRE ARGS_SUB &TestClass::cbConstRef, sharedObj);\
  auto subSharedClassValue = SUB(ARGS_PRE ARGS_SUB &TestClass::cbValue, sharedObj);\
  auto subSharedClassConstEvent = SUB(ARGS_PRE ARGS_SUB &TestClass::cbConstEvent, sharedObj);\
  auto subSharedClassEvent = SUB(ARGS_PRE ARGS_SUB &TestClass::cbEvent, sharedObj);\
  \
  auto subConstSharedClassConstPtr = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbConstPtr, sharedObj);\
  auto subConstSharedClassPtr = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbPtr, sharedObj);\
  auto subConstSharedClassConstRef = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbConstRef, sharedObj);\
  auto subConstSharedClassValue = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbValue, sharedObj);\
  auto subConstSharedClassConstEvent = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbConstEvent, sharedObj);\
  auto subConstSharedClassEvent = SUB(ARGS_PRE ARGS_SUB &TestClass::constCbEvent, sharedObj);\
  \
  auto subNoCaptureLambdaConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    [](const MsgConstPtr& m){noCaptureLambdaConstPtrFrame = FRAME_PTR;});\
  auto subNoCaptureLambdaPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    [](const MsgPtr& m){noCaptureLambdaPtrFrame = FRAME_PTR;});\
  auto subNoCaptureLambdaConstRef = SUB(ARGS_PRE ARGS_SUB\
    (void(*)(const Msg&)) [](const Msg& m){noCaptureLambdaConstRefFrame = FRAME_REF;});\
  auto subNoCaptureLambdaValue = SUB(ARGS_PRE ARGS_SUB\
    (void(*)(Msg)) [](Msg m){noCaptureLambdaValueFrame = FRAME_REF;});\
  auto subNoCaptureLambdaConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (void(*)(const ros::MessageEvent<Msg const>&))\
      [](const ros::MessageEvent<Msg const>& m){noCaptureLambdaConstEventFrame = FRAME_EVENT_PTR;});\
  auto subNoCaptureLambdaEvent = SUB(ARGS_PRE ARGS_SUB\
    (void(*)(const ros::MessageEvent<Msg>&))\
      [](const ros::MessageEvent<Msg>& m){noCaptureLambdaEventFrame = FRAME_EVENT_PTR;});\
  \
  auto subRefCaptureLambdaConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    [&](const MsgConstPtr& m){refCaptureLambdaConstPtrFrame = FRAME_PTR;});\
  auto subRefCaptureLambdaPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    [&](const MsgPtr& m){refCaptureLambdaPtrFrame = FRAME_PTR;});\
  auto subRefCaptureLambdaConstRef = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const Msg&)>)[&](const Msg& m){refCaptureLambdaConstRefFrame = FRAME_REF;});\
  auto subRefCaptureLambdaValue = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(Msg)>)[&](Msg m){refCaptureLambdaValueFrame = FRAME_REF;});\
  auto subRefCaptureLambdaConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg const>&)>)\
      [&](const ros::MessageEvent<Msg const>& m){refCaptureLambdaConstEventFrame = FRAME_EVENT_PTR;});\
  auto subRefCaptureLambdaEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg>&)>)\
      [&](const ros::MessageEvent<Msg>& m){refCaptureLambdaEventFrame = FRAME_EVENT_PTR;});\
  \
  auto subValueCaptureLambdaConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    [=](const MsgConstPtr& m){valueCaptureLambdaConstPtrFrame = FRAME_PTR;});\
  auto subValueCaptureLambdaPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    [=](const MsgPtr& m){valueCaptureLambdaPtrFrame = FRAME_PTR;});\
  auto subValueCaptureLambdaConstRef = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const Msg&)>)[=](const Msg& m){valueCaptureLambdaConstRefFrame = FRAME_REF;});\
  auto subValueCaptureLambdaValue = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(Msg)>)[=](Msg m){valueCaptureLambdaValueFrame = FRAME_REF;});\
  auto subValueCaptureLambdaConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg const>&)>)\
      [=](const ros::MessageEvent<Msg const>& m){valueCaptureLambdaConstEventFrame = FRAME_EVENT_PTR;});\
  auto subValueCaptureLambdaEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg>&)>)\
      [=](const ros::MessageEvent<Msg>& m){valueCaptureLambdaEventFrame = FRAME_EVENT_PTR;});\
  \
  auto subLambdaCbConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB lambda##HEADER##CbConstPtr);\
  auto subLambdaCbPtr = SUB<Msg>(ARGS_PRE ARGS_SUB lambda##HEADER##CbPtr);\
  auto subLambdaCbConstRef = SUB(ARGS_PRE ARGS_SUB (void(*)(const Msg&))lambda##HEADER##CbConstRef);\
  auto subLambdaCbValue = SUB(ARGS_PRE ARGS_SUB (void(*)(Msg))lambda##HEADER##CbValue);\
  auto subLambdaCbConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (void(*)(const ros::MessageEvent<Msg const>&))lambda##HEADER##CbConstEvent);\
  auto subLambdaCbEvent = SUB(ARGS_PRE ARGS_SUB (void(*)(const ros::MessageEvent<Msg>&))lambda##HEADER##CbEvent);\
  \
  auto subLambdaHintsConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB lambda##HEADER##HintsConstPtr, ros::TransportHints());\
  auto subLambdaHintsPtr = SUB<Msg>(ARGS_PRE ARGS_SUB lambda##HEADER##HintsPtr, ros::TransportHints());\
  auto subLambdaHintsConstRef = SUB(ARGS_PRE ARGS_SUB\
    (void(*)(const Msg&))lambda##HEADER##HintsConstRef, ros::TransportHints());\
  auto subLambdaHintsValue = SUB(ARGS_PRE ARGS_SUB (void(*)(Msg))lambda##HEADER##HintsValue, ros::TransportHints());\
  auto subLambdaHintsConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (void(*)(const ros::MessageEvent<Msg const>&))lambda##HEADER##HintsConstEvent, ros::TransportHints());\
  auto subLambdaHintsEvent = SUB(ARGS_PRE ARGS_SUB\
    (void(*)(const ros::MessageEvent<Msg>&))lambda##HEADER##HintsEvent, ros::TransportHints());\
  \
  auto subClassBindConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB boost::bind(&TestClass::cbConstPtr, &obj2, boost::placeholders::_1));\
  auto subClassBindPtr = SUB<Msg>(ARGS_PRE ARGS_SUB boost::bind(&TestClass::cbPtr, &obj2, boost::placeholders::_1));\
  auto subClassBindConstRef = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const Msg&)>)boost::bind(&TestClass::cbConstRef, &obj2, boost::placeholders::_1));\
  auto subClassBindValue = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(Msg)>)boost::bind(&TestClass::cbValue, &obj2, boost::placeholders::_1));\
  auto subClassBindConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg const>&)>)\
      boost::bind(&TestClass::cbConstEvent, &obj2, boost::placeholders::_1));\
  auto subClassBindEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg>&)>)\
      boost::bind(&TestClass::cbEvent, &obj2, boost::placeholders::_1));\
  \
  auto subConstClassBindConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    boost::bind(&TestClass::constCbConstPtr, &obj2, boost::placeholders::_1));\
  auto subConstClassBindPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    boost::bind(&TestClass::constCbPtr, &obj2, boost::placeholders::_1));\
  auto subConstClassBindConstRef = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const Msg&)>)boost::bind(&TestClass::constCbConstRef, &obj2, boost::placeholders::_1));\
  auto subConstClassBindValue = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(Msg)>)boost::bind(&TestClass::constCbValue, &obj2, boost::placeholders::_1));\
  auto subConstClassBindConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg const>&)>)\
      boost::bind(&TestClass::constCbConstEvent, &obj2, boost::placeholders::_1));\
  auto subConstClassBindEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg>&)>)\
      boost::bind(&TestClass::constCbEvent, &obj2, boost::placeholders::_1));\
  \
  auto subSharedClassBindConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB std::bind(&TestClass::cbConstPtr, &obj3, std::placeholders::_1));\
  auto subSharedClassBindPtr = SUB<Msg>(ARGS_PRE ARGS_SUB std::bind(&TestClass::cbPtr, &obj3, std::placeholders::_1));\
  auto subSharedClassBindConstRef = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const Msg&)>)std::bind(&TestClass::cbConstRef, &obj3, std::placeholders::_1));\
  auto subSharedClassBindValue = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(Msg)>)std::bind(&TestClass::cbValue, &obj3, std::placeholders::_1));\
  auto subSharedClassBindConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg const>&)>)\
      std::bind(&TestClass::cbConstEvent, &obj3, std::placeholders::_1));\
  auto subSharedClassBindEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg>&)>)\
      std::bind(&TestClass::cbEvent, &obj3, std::placeholders::_1));\
  \
  auto subConstSharedClassBindConstPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    std::bind(&TestClass::constCbConstPtr, &obj3, std::placeholders::_1));\
  auto subConstSharedClassBindPtr = SUB<Msg>(ARGS_PRE ARGS_SUB\
    std::bind(&TestClass::constCbPtr, &obj3, std::placeholders::_1));\
  auto subConstSharedClassBindConstRef = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const Msg&)>)std::bind(&TestClass::constCbConstRef, &obj3, std::placeholders::_1));\
  auto subConstSharedClassBindValue = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(Msg)>)std::bind(&TestClass::constCbValue, &obj3, std::placeholders::_1));\
  auto subConstSharedClassBindConstEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg const>&)>)\
      std::bind(&TestClass::constCbConstEvent, &obj3, std::placeholders::_1));\
  auto subConstSharedClassBindEvent = SUB(ARGS_PRE ARGS_SUB\
    (boost::function<void(const ros::MessageEvent<Msg>&)>)\
      std::bind(&TestClass::constCbEvent, &obj3, std::placeholders::_1));\
  \
  auto subBoostLambdaConstPtr = SUB(ARGS_PRE ARGS_SUB boost##HEADER##LambdaConstPtr);\
  auto subBoostLambdaPtr = SUB(ARGS_PRE ARGS_SUB boost##HEADER##LambdaPtr);\
  auto subBoostLambdaConstRef = SUB(ARGS_PRE ARGS_SUB boost##HEADER##LambdaConstRef);\
  auto subBoostLambdaValue = SUB(ARGS_PRE ARGS_SUB boost##HEADER##LambdaValue);\
  auto subBoostLambdaConstEvent = SUB(ARGS_PRE ARGS_SUB boost##HEADER##LambdaConstEvent);\
  auto subBoostLambdaEvent = SUB(ARGS_PRE ARGS_SUB boost##HEADER##LambdaEvent);\
  \
  ros::SubscribeOptions ops;\
  ops.initByFullCallbackType<const MsgConstPtr &>(ARGS_SUB &options##HEADER##CbConstPtr);\
  auto subOptionsConstPtr = SUB<Msg>(ARGS_PRE ops);\
  ops.initByFullCallbackType<const MsgPtr &>(ARGS_SUB &options##HEADER##CbPtr);\
  auto subOptionsPtr = SUB<Msg>(ARGS_PRE ops);\
  ops.initByFullCallbackType<const Msg&>(ARGS_SUB &options##HEADER##CbConstRef);\
  auto subOptionsConstRef = SUB<Msg>(ARGS_PRE ops);\
  ops.initByFullCallbackType<Msg>(ARGS_SUB &options##HEADER##CbValue);\
  auto subOptionsValue = SUB<Msg>(ARGS_PRE ops);\
  ops.initByFullCallbackType<const ros::MessageEvent<Msg const>&>(ARGS_SUB &options##HEADER##CbConstEvent);\
  auto subOptionsConstEvent = SUB<Msg>(ARGS_PRE ops);\
  ops.initByFullCallbackType<const ros::MessageEvent<Msg>&>(ARGS_SUB &options##HEADER##CbEvent);\
  auto subOptionsEvent = SUB<Msg>(ARGS_PRE ops);
