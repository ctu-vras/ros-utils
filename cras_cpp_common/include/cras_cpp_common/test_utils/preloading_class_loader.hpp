/**
 * \file
 * \brief A hack that allows forcing a pluginlib classloader to load a compiled-in class instead of seeking for it in
 *        plugins using the standard rospack-based algorithm.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <class_loader/class_loader.hpp>
#include <pluginlib/class_loader.hpp>

#include <cras_cpp_common/type_utils.hpp>

/*
 * In the classes where you need to use this classloader, insert the following code at the very beginning of the CPP
 * file:
 * 
// HACK: we need to alter the private lowlever_class_loader_ of pluginlib::ClassLoader
#define private public
#include <pluginlib/class_loader.hpp>
#undef private
 *
 */

/**
 * \brief This pluginlib classloader alternative can load compiled-in classes and provide them as if they were loaded
 *        via the standard pluginlib search mechanism - without even triggering the search.
 * \tparam Impl The class to be preloaded.
 * \tparam Base The base class for which pluginlib is looking.
 */
template <typename Impl, typename Base>
class PreloadingClassLoader : public class_loader::ClassLoader
{
protected:
  /**
   * \brief Construct the class loader.
   * \note Do not call this constructor directly. Instead, let the `preload()` method construct it.
   */
  PreloadingClassLoader() : ClassLoader("", true)
  {
    const auto implType = cras::getTypeName<Impl>();
    const auto baseType = cras::getTypeName<Base>();
    auto metaObject = new class_loader::impl::MetaObject<Impl, Base>(implType, baseType);
    metaObject->addOwningClassLoader(this);
    class_loader::impl::getFactoryMapForBaseClass<Base>()[implType] = metaObject;
  }

public:
  ~PreloadingClassLoader() override
  {
    const auto implType = cras::getTypeName<Impl>();
    auto metaObject = class_loader::impl::getFactoryMapForBaseClass<Base>()[implType];
    metaObject->removeOwningClassLoader(this);
    class_loader::impl::getFactoryMapForBaseClass<Base>().erase(implType);
    delete metaObject;
  }

  /**
   * \brief "Install" the preloading classloader into the given classloader.
   * \param[in] pluginlibType Name of the type that would be search for by pluginlib.
   * \param[in] loader The pluginlib loader into which the class should be injected.
   */
  static void preload(const std::string& pluginlibType, pluginlib::ClassLoader<Base>& loader)
  {
    const auto implType = cras::getTypeName<Impl>();
    const auto baseType = cras::getTypeName<Base>();
    loader.lowlevel_class_loader_.active_class_loaders_[implType] = new PreloadingClassLoader<Impl, Base>();
    loader.classes_available_.insert({pluginlibType, {pluginlibType, implType, baseType, "", "", "", ""}});
    loader.classes_available_.at(pluginlibType).resolved_library_path_ = implType;
  }

  /**
   * \brief Revert the effect of `preload()`.
   * \param[in] pluginlibType 
   * \param[in] loader The pluginlib classloader from which the class should be "un-injected".
   * \note All instances of the preloaded class should be destroyed prior to calling this function, otherwise segfaults
   *       can happen.
   */
  static void unPreload(const std::string& pluginlibType, pluginlib::ClassLoader<Base>& loader)
  {
    loader.unloadLibraryForClass(pluginlibType);
  }
};