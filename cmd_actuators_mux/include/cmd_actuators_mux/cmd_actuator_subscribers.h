#ifndef CMD_ACTUATORS_MUX_CMD_ACTUATORS_SUBSCRIBER_H_
#define CMD_ACTUATORS_MUX_CMD_ACTUATORS_SUBSCRIBER_H_

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

namespace cmd_actuator_mux {

class CmdActuatorSubscribers
{
public:

  class CmdActuatorSubs
  {
  public:
    unsigned int           idx;          /**< Index; assigned according to the order on YAML file */
    std::string            name;         /**< Descriptive name; must be unique to this subscriber */
    std::string            topic;        /**< The name of the topic */
    ros::Subscriber        subs;         /**< The subscriber itself */
    ros::Timer             timer;        /**< No incoming messages timeout */
    double                 timeout;      /**< Timer's timeout, in seconds  */
    unsigned int           priority;     /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string            short_desc;   /**< Short description (optional) */
    bool                   active;       /**< Whether this source is active */

    CmdActuatorSubs(unsigned int idx) : idx(idx), active(false) { };
    ~CmdActuatorSubs() { }

    /** Fill attributes with a YAML node content */
    void operator << (const YAML::Node& node);
  };

  CmdActuatorSubscribers() { }
  ~CmdActuatorSubscribers() { }

  std::vector<std::shared_ptr<CmdActuatorSubs>>::size_type size() { return list.size(); };
  std::shared_ptr<CmdActuatorSubs>& operator [] (unsigned int idx) { return list[idx]; };

  /**
   * @brief Configures the subscribers from a yaml file.
   *
   * @exception FileNotFoundException : yaml file not found
   * @exception YamlException : problem parsing the yaml
   * @exception EmptyCfgException : empty configuration file
   * @param node : node holding all the subscriber configuration
   */
  void configure(const YAML::Node& node);

  unsigned int allowed;

private:
  std::vector<std::shared_ptr<CmdActuatorSubs>> list;
};

}  //  namespace namespace cmd_actuator_mux

#endif /* CMD_ACTUATORS_MUX_CMD_ACTUATORS_SUBSCRIBER_H_ */
