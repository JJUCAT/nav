/*
 *  Copyright 2018, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met: *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  abstract_plugin_manager.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#include <navit_core/abstract_plugin_manager.h>
#include <XmlRpcException.h>

namespace navit_core {

template <typename PluginType>
AbstractPluginManager<PluginType>::AbstractPluginManager(
    const std::string &param_name,
    const loadPluginFunction &loadPlugin,
    const initPluginFunction &initPlugin
)
  : param_name_(param_name), loadPlugin_(loadPlugin), initPlugin_(initPlugin)
{
}

template <typename PluginType>
AbstractPluginManager<PluginType>::AbstractPluginManager(
    const std::string &param_name,
    const loadPluginFunction &loadPlugin,
    const initPluginFunction &initPlugin,
    const ros::NodeHandle& nh)
  : param_name_(param_name), loadPlugin_(loadPlugin), initPlugin_(initPlugin), nh_(nh)
{
}

template <typename PluginType>
bool AbstractPluginManager<PluginType>::loadPlugins()
{
  XmlRpc::XmlRpcValue plugin_param_list;
  if(!nh_.getParam(param_name_, plugin_param_list))
  {
    ROS_WARN_STREAM("No " << param_name_ << " plugins configured! - Use the param \"" << param_name_ << "\", "
        "which must be a list of tuples with a name and a type.");
    return false;
  }

  try
  {
    for (int i = 0; i < plugin_param_list.size(); i++)
    {
      XmlRpc::XmlRpcValue elem = plugin_param_list[i];

      std::string name = elem["name"];
      std::string type = elem["type"];

      if (plugins_.find(name) != plugins_.end())
      {
        ROS_ERROR_STREAM("The plugin \"" << name << "\" has already been loaded! Names must be unique!");
        return false;
      }
      typename PluginType::Ptr plugin_ptr = loadPlugin_(type);
      if(plugin_ptr && initPlugin_(name, plugin_ptr))
      {

        plugins_.insert(
            std::pair<std::string, typename PluginType::Ptr>(name, plugin_ptr));

        plugins_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping
        names_.push_back(name);

        ROS_INFO_STREAM("The plugin with the type \"" << type << "\" has been loaded successfully under the name \""
                                                       << name << "\".");
      }
      else
      {
        ROS_ERROR_STREAM("Could not load the plugin with the name \""
                             << name << "\" and the type \"" << type << "\"!");
      }
    }
  }
  catch (XmlRpc::XmlRpcException &e)
  {
    ROS_ERROR_STREAM("Invalid parameter structure. The \""<< param_name_ << "\" parameter has to be a list of structs "
                         << "with fields \"name\" and \"type\" of !");
    ROS_ERROR_STREAM(e.getMessage());
    return false;
  }
  // is there any plugin in the map?
  return plugins_.empty() ? false : true;
}

template <typename PluginType>
const std::vector<std::string>& AbstractPluginManager<PluginType>::getLoadedNames()
{
  return names_;
}

template <typename PluginType>
bool AbstractPluginManager<PluginType>::hasPlugin(const std::string &name)
{
  return static_cast<bool>(plugins_.count(name)); // returns 1 or 0;
}

template <typename PluginType>
std::string AbstractPluginManager<PluginType>::getType(const std::string &name)
{
  std::map<std::string, std::string>::iterator iter = plugins_type_.find(name);
  return iter->second;
}


template <typename PluginType>
typename PluginType::Ptr AbstractPluginManager<PluginType>::getPlugin(const std::string &name)
{
  typename std::map<std::string, typename PluginType::Ptr>::iterator new_plugin
      = plugins_.find(name);
  if(new_plugin != plugins_.end())
  {
    ROS_DEBUG_STREAM("Found plugin with the name \"" << name << "\".");
    return new_plugin->second;
  }
  else
  {
    ROS_WARN_STREAM("The plugin with the name \"" << name << "\" has not yet been loaded!");
    return typename PluginType::Ptr(); // return null ptr
  }
}

template <typename PluginType>
void AbstractPluginManager<PluginType>::clearPlugins() {
  plugins_.clear();
  plugins_type_.clear();
  names_.clear();
}

}


