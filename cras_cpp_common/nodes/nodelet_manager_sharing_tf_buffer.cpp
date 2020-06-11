#include <bondcpp/bond.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <pluginlib/class_loader.h>

#include <boost/ptr_container/ptr_map.hpp>

namespace cras
{

// This class is a copy-paste of LoaderROS from https://github.com/ros/nodelet_core/blob/indigo-devel/nodelet/src/loader.cpp
class LoaderROS
{
public:
  LoaderROS(nodelet::Loader* parent, const ros::NodeHandle& nh)
      : parent_(parent)
      , nh_(nh)
      , bond_spinner_(1, &bond_callback_queue_)
  {
    load_server_ = nh_.advertiseService("load_nodelet", &LoaderROS::serviceLoad, this);
    unload_server_ = nh_.advertiseService("unload_nodelet", &LoaderROS::serviceUnload, this);
    list_server_ = nh_.advertiseService("list", &LoaderROS::serviceList, this);

    bond_spinner_.start();
  }

private:
  bool serviceLoad(nodelet::NodeletLoad::Request& req,
                   nodelet::NodeletLoad::Response& res)
  {
    boost::mutex::scoped_lock lock(lock_);

    // build map
    nodelet::M_string remappings;
    if (req.remap_source_args.size() != req.remap_target_args.size())
    {
      ROS_ERROR("Bad remapppings provided, target and source of different length");
    }
    else
    {
      for (size_t i = 0; i < req.remap_source_args.size(); ++i)
      {
        remappings[ros::names::resolve(req.remap_source_args[i])] = ros::names::resolve(req.remap_target_args[i]);
        ROS_DEBUG("%s:%s\n", ros::names::resolve(req.remap_source_args[i]).c_str(), remappings[ros::names::resolve(req.remap_source_args[i])].c_str());
      }
    }

    res.success = parent_->load(req.name, req.type, remappings, req.my_argv);

    // If requested, create bond to sister process
    if (res.success && !req.bond_id.empty())
    {
      bond::Bond* bond = new bond::Bond(nh_.getNamespace() + "/bond", req.bond_id);
      bond_map_.insert(req.name, bond);
      bond->setCallbackQueue(&bond_callback_queue_);
      bond->setBrokenCallback(boost::bind(&LoaderROS::unload, this, req.name));
      bond->start();
    }
    return res.success;
  }

  bool serviceUnload(nodelet::NodeletUnload::Request& req,
                     nodelet::NodeletUnload::Response& res)
  {
    res.success = unload(req.name);
    return res.success;
  }

  bool unload(const std::string& name)
  {
    boost::mutex::scoped_lock lock(lock_);

    bool success = parent_->unload(name);
    if (!success)
    {
      ROS_ERROR("Failed to find nodelet with name '%s' to unload.", name.c_str());
      return success;
    }

    // break the bond, if there is one
    M_stringToBond::iterator it = bond_map_.find(name);
    if (it != bond_map_.end()) {
      // disable callback for broken bond, as we are breaking it intentially now
      it->second->setBrokenCallback(boost::function<void(void)>());
      // erase (and break) bond
      bond_map_.erase(name);
    }

    return success;
  }

  bool serviceList(nodelet::NodeletList::Request&,
                   nodelet::NodeletList::Response& res)
  {
    res.nodelets = parent_->listLoadedNodelets();
    return true;
  }

  nodelet::Loader* parent_;
  ros::NodeHandle nh_;
  ros::ServiceServer load_server_;
  ros::ServiceServer unload_server_;
  ros::ServiceServer list_server_;

  boost::mutex lock_;

  ros::CallbackQueue bond_callback_queue_;
  ros::AsyncSpinner bond_spinner_;
  typedef boost::ptr_map<std::string, bond::Bond> M_stringToBond;
  M_stringToBond bond_map_;
};

class NodeletManagerSharingTfBuffer : StatefulNodelet
{
public:
  void init();
  virtual ~NodeletManagerSharingTfBuffer() = default;  // we need to be polymorphic to use dynamic_cast
protected:
  boost::shared_ptr<nodelet::Nodelet> createInstance(const std::string & lookup_name);
private:
  typedef pluginlib::ClassLoader<nodelet::Nodelet> ClassLoader;

  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::unique_ptr<nodelet::Loader> loader;
  std::unique_ptr<ClassLoader> classLoader;
  std::unique_ptr<LoaderROS> loaderRos;
  ros::NodeHandle nh;
};

void NodeletManagerSharingTfBuffer::init() {
  this->buffer.reset(new NodeletAwareTFBuffer(this));
  this->listener.reset(new tf2_ros::TransformListener(*this->buffer));
  this->classLoader.reset(new ClassLoader("nodelet", "nodelet::Nodelet"));
  this->classLoader->refreshDeclaredClasses();
  auto createInstance = boost::bind(&NodeletManagerSharingTfBuffer::createInstance, this, _1);
  this->loader.reset(new nodelet::Loader(createInstance));
  nh = ros::NodeHandle("~");
  this->loaderRos.reset(new LoaderROS(this->loader.get(), nh));
}

boost::shared_ptr<nodelet::Nodelet> NodeletManagerSharingTfBuffer::createInstance(const std::string &lookup_name) {
  const auto ptr = this->classLoader->createInstance(lookup_name);
  if (ptr == nullptr)
    return nullptr;

  auto* tfNodelet = dynamic_cast<NodeletWithSharedTfBuffer*>(ptr.get());
  if (tfNodelet != nullptr)
    tfNodelet->setBuffer(this->buffer);
  return ptr;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manager");
  cras::NodeletManagerSharingTfBuffer manager;
  manager.init();
  ros::spin();
}

